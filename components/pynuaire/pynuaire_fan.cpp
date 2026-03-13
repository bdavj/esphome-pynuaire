#include "pynuaire_fan.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pynuaire {

static const char *const TAG = "pynuaire";

// ============================================================
// ESPHome lifecycle
// ============================================================

void PyNuaireFan::setup() {
  this->target_level_  = this->default_level_;
  this->current_level_ = 0;
  this->synced_        = false;
  this->sync_count_    = 0;
  this->ctr16_         = 0x80;
  this->have_seen_rx_  = false;
  this->rx_buf_.reserve(RX_BUF_MAX);

  // Publish initial state to HA
  auto call = this->make_call();
  call.set_state(true);
  call.set_speed(this->default_level_);
  call.perform();
}

void PyNuaireFan::loop() {
  this->poll_serial_();

  uint32_t now = millis();

  // Scheduled reply to motor
  if (this->pending_tx_at_ms_ != 0 && now >= this->pending_tx_at_ms_) {
    this->send_packet_("reply");
  }

  // Keepalive if motor has gone quiet
  if (this->have_seen_rx_ && this->pending_tx_at_ms_ == 0) {
    if (this->last_tx_ms_ != 0 && (now - this->last_tx_ms_) >= KEEPALIVE_MS) {
      this->send_packet_("keepalive");
    }
  }

  // Alive watchdog — mark dead if no RX for ALIVE_TIMEOUT_MS
  if (this->alive_sensor_ != nullptr && this->have_seen_rx_) {
    bool alive = (now - this->last_rx_ms_) < ALIVE_TIMEOUT_MS;
    this->alive_sensor_->publish_state(alive);
  }
}

void PyNuaireFan::dump_config() {
  LOG_FAN("", "PyNuaire Drimaster Fan", this);
  ESP_LOGCONFIG(TAG, "  Default level: %d", this->default_level_);
}

// ============================================================
// Fan traits / control
// ============================================================

fan::FanTraits PyNuaireFan::get_traits() {
  auto traits = fan::FanTraits(false, true, false, 6);
  return traits;
}

void PyNuaireFan::control(const fan::FanCall &call) {
  if (call.get_speed().has_value()) {
    int new_level = (int) call.get_speed().value();
    if (new_level < 1) new_level = 1;
    if (new_level > 6) new_level = 6;
    if (new_level != this->target_level_) {
      ESP_LOGI(TAG, "HA requested level L%d -> L%d", this->target_level_, new_level);
      this->target_level_ = new_level;
      // Allow immediate stepping
      if (this->sync_count_ < 3) this->sync_count_ = 3;
    }
  }

  // If turned off, go to L1 (minimum — fan must always run)
  if (call.get_state().has_value() && !call.get_state().value()) {
    this->target_level_ = 1;
    auto restore = this->make_call();
    restore.set_state(true);
    restore.set_speed(1);
    restore.perform();
    return;
  }

  this->state  = true;
  this->speed  = this->target_level_;
  this->publish_state();
}

// ============================================================
// Serial polling
// ============================================================

void PyNuaireFan::poll_serial_() {
  while (this->available()) {
    uint8_t b;
    this->read_byte(&b);
    this->rx_buf_.push_back(b);
    if (this->rx_buf_.size() > RX_BUF_MAX) {
      this->rx_buf_.erase(this->rx_buf_.begin(),
                          this->rx_buf_.begin() + (RX_BUF_MAX / 2));
    }
  }

  uint8_t pkt[30];
  while (this->find_next_rx_packet_(pkt)) {
    this->handle_rx_packet_(pkt);
  }
}

// ============================================================
// Frame scanning: find 1B 1B ... 0D, nibble-decode, checksum
// ============================================================

bool PyNuaireFan::find_next_rx_packet_(uint8_t *out_pkt_30) {
  while (true) {
    // Find 1B 1B header
    size_t start = std::string::npos;
    for (size_t i = 0; i + 1 < this->rx_buf_.size(); i++) {
      if (this->rx_buf_[i] == 0x1B && this->rx_buf_[i + 1] == 0x1B) {
        start = i;
        break;
      }
    }
    if (start == std::string::npos) {
      // No header — keep last byte in case it's the start of one
      if (this->rx_buf_.size() > 1)
        this->rx_buf_.erase(this->rx_buf_.begin(), this->rx_buf_.end() - 1);
      return false;
    }

    // Discard anything before the header
    if (start > 0)
      this->rx_buf_.erase(this->rx_buf_.begin(), this->rx_buf_.begin() + start);

    // Find 0D terminator after header
    size_t end = std::string::npos;
    for (size_t i = 2; i < this->rx_buf_.size(); i++) {
      if (this->rx_buf_[i] == 0x0D) {
        end = i;
        break;
      }
    }
    if (end == std::string::npos) {
      // Stale header with no terminator — skip
      if (this->rx_buf_.size() > 200)
        this->rx_buf_.erase(this->rx_buf_.begin(), this->rx_buf_.begin() + 2);
      return false;
    }

    // Extract body between header and terminator
    const uint8_t *body = this->rx_buf_.data() + 2;
    size_t body_len = end - 2;

    // Consume from buffer
    this->rx_buf_.erase(this->rx_buf_.begin(), this->rx_buf_.begin() + end + 1);

    // Decode and validate
    if (!decode_wire_(body, body_len, out_pkt_30))
      continue;
    if (!verify_checksum_(out_pkt_30))
      continue;
    if (out_pkt_30[0x00] != 0x81)
      continue;  // only want motor→ctrl packets

    return true;
  }
}

// ============================================================
// RX packet handling
// ============================================================

void PyNuaireFan::handle_rx_packet_(const uint8_t *pkt) {
  this->have_seen_rx_ = true;
  this->last_rx_ms_   = millis();

  if (this->alive_sensor_ != nullptr)
    this->alive_sensor_->publish_state(true);

  uint8_t level_byte  = pkt[0x05];
  uint8_t b16         = pkt[0x16];
  int     motor_level = (level_byte >= 1 && level_byte <= 6) ? (int) level_byte : 0;

  ESP_LOGD(TAG, "RX level=L%d B08=0x%02X B16=0x%02X", motor_level, pkt[0x08], b16);

  // Sync: adopt motor's level on first contact
  if (!this->synced_ && motor_level > 0) {
    this->current_level_ = motor_level;
    this->synced_        = true;
    this->sync_count_    = 0;
    ESP_LOGI(TAG, "Synced to motor L%d", motor_level);

    // Update HA to reflect actual level
    this->speed = motor_level;
    this->publish_state();
  }

  // Track motor-acknowledged level changes
  if (motor_level > 0 && motor_level != this->current_level_ && this->synced_) {
    ESP_LOGI(TAG, "Motor level: L%d -> L%d", this->current_level_, motor_level);
    this->current_level_ = motor_level;
    this->sync_count_    = 0;

    // Update HA
    this->speed = motor_level;
    this->publish_state();
  }

  // Publish synced state: true when motor is running at our target level
  if (this->synced_sensor_ != nullptr) {
    bool in_sync = this->synced_ && (this->current_level_ == this->target_level_);
    this->synced_sensor_->publish_state(in_sync);
  }

  // B16 counter: motor sent N, we reply N-1
  this->ctr16_ = (b16 - 1) & 0xFF;

  // Schedule TX reply
  this->pending_tx_at_ms_ = millis() + RESPONSE_DELAY_MS;
}

// ============================================================
// TX packet construction and send
// ============================================================

void PyNuaireFan::send_packet_(const char *reason) {
  uint8_t pkt[30];
  this->build_tx_packet_(pkt);

  uint8_t wire[2 + 30 * 2 + 1];
  size_t  wire_len = 0;
  encode_wire_(pkt, wire, &wire_len);

  this->write_array(wire, wire_len);
  this->last_tx_ms_       = millis();
  this->pending_tx_at_ms_ = 0;
  this->sync_count_++;

  ESP_LOGD(TAG, "TX (%s) level=L%d B18=0x%02X B16=0x%02X",
           reason, pkt[0x05], pkt[0x18], pkt[0x16]);
}

void PyNuaireFan::build_tx_packet_(uint8_t *pkt) {
  // Determine what level to send
  int send_level = this->current_level_;
  if (send_level == 0) send_level = this->default_level_;

  if (this->synced_ && this->sync_count_ >= 3) {
    // Step one level at a time toward target
    if (send_level < this->target_level_)      send_level = send_level + 1;
    else if (send_level > this->target_level_) send_level = send_level - 1;
  }

  // B18=0x02 on *first* packet with a new level
  bool level_changing = (this->last_sent_level_ != 0 && send_level != this->last_sent_level_);

  memset(pkt, 0, 30);
  pkt[0x00] = 0x82;           // direction: ctrl→motor
  pkt[0x01] = 0xAB;
  pkt[0x02] = 0x0C;
  pkt[0x04] = 0x01;
  pkt[0x05] = (uint8_t) send_level;
  pkt[0x16] = this->ctr16_;
  pkt[0x17] = 0x81;
  pkt[0x18] = level_changing ? 0x02 : 0x00;
  pkt[0x1B] = 0x02;
  pkt[0x1D] = compute_checksum_(pkt);

  this->last_sent_level_ = send_level;
  this->ctr16_           = (this->ctr16_ - 1) & 0xFF;
}

// ============================================================
// Wire encoding / decoding
// ============================================================

bool PyNuaireFan::decode_wire_(const uint8_t *wire, size_t wire_len, uint8_t *out_30) {
  // Strip any odd trailing byte
  size_t len = wire_len;
  if (len % 2 != 0) len--;
  if (len < 60) return false;

  for (size_t i = 0; i < 30; i++) {
    uint8_t hi = wire[i * 2]     & 0x0F;
    uint8_t lo = wire[i * 2 + 1] & 0x0F;
    out_30[i] = (hi << 4) | lo;
  }
  return true;
}

void PyNuaireFan::encode_wire_(const uint8_t *pkt_30, uint8_t *out_wire, size_t *out_len) {
  size_t pos = 0;
  out_wire[pos++] = 0x1B;
  out_wire[pos++] = 0x1B;
  for (int i = 0; i < 30; i++) {
    out_wire[pos++] = 0x30 | ((pkt_30[i] >> 4) & 0x0F);
    out_wire[pos++] = 0x30 | (pkt_30[i] & 0x0F);
  }
  out_wire[pos++] = 0x0D;
  *out_len = pos;
}

uint8_t PyNuaireFan::compute_checksum_(const uint8_t *pkt_30) {
  uint16_t sum = 0;
  for (int i = 0; i < 0x1D; i++) sum += pkt_30[i];
  return (uint8_t) ((-sum) & 0xFF);
}

bool PyNuaireFan::verify_checksum_(const uint8_t *pkt_30) {
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++) sum += pkt_30[i];
  return (sum & 0xFF) == 0;
}

}  // namespace pynuaire
}  // namespace esphome
