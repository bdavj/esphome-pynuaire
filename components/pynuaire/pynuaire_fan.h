#pragma once

#include "esphome/core/component.h"
#include "esphome/components/fan/fan.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>

namespace esphome {
namespace pynuaire {

/*
 * Nuaire Drimaster dMEV fan controller — new UART protocol.
 *
 * Wire format:  1B 1B <nibble-encoded 30 bytes> 0D
 * Baud:         57600 8N1, signal INVERTED (use inverted: true on both rx_pin / tx_pin in YAML)
 *
 * Motor initiates every ~100ms.  We reply within ~17ms.
 * B05  = fan level (0x01–0x06)
 * B16  = shared counter (each side replies with received - 1)
 * B18  = 0x02 on the *first* packet of a new level, else 0x00
 */
class PyNuaireFan : public Component, public fan::Fan, public uart::UARTDevice {
 public:
  // ---- ESPHome lifecycle ----
  void setup() override;
  void loop() override;
  void dump_config() override;

  // ---- Fan traits / control ----
  fan::FanTraits get_traits() override;
  void control(const fan::FanCall &call) override;

  // ---- Config setters (called from fan.py to_code) ----
  void set_default_level(int level) { this->default_level_ = level; }
  void set_alive_sensor(binary_sensor::BinarySensor *s) { this->alive_sensor_ = s; }
  void set_synced_sensor(binary_sensor::BinarySensor *s) { this->synced_sensor_ = s; }

 protected:
  // ---- Internal protocol helpers ----
  void poll_serial_();
  bool find_next_rx_packet_(uint8_t *out_pkt_30);
  void handle_rx_packet_(const uint8_t *pkt);
  void send_packet_(const char *reason);
  void build_tx_packet_(uint8_t *pkt_30);

  static bool decode_wire_(const uint8_t *wire, size_t wire_len, uint8_t *out_30);
  static void encode_wire_(const uint8_t *pkt_30, uint8_t *out_wire, size_t *out_len);
  static uint8_t compute_checksum_(const uint8_t *pkt_30);  // checksum of bytes 0x00–0x1C
  static bool verify_checksum_(const uint8_t *pkt_30);

  // ---- Config ----
  int default_level_{3};

  // ---- Sync / level state ----
  int  target_level_{3};   // what HA/user wants
  int  current_level_{0};  // last level reported by motor (0 = not yet synced)
  int  last_sent_level_{0};
  bool synced_{false};
  int  sync_count_{0};

  // ---- Protocol counters ----
  uint8_t  ctr16_{0x80};

  // ---- Status sensors ----
  binary_sensor::BinarySensor *alive_sensor_{nullptr};
  binary_sensor::BinarySensor *synced_sensor_{nullptr};
  int8_t last_alive_pub_{-1};   // -1=never, 0=false, 1=true
  int8_t last_synced_pub_{-1};
  uint32_t out_of_sync_since_ms_{0};  // 0 = currently in sync

  // ---- Timing ----
  uint32_t pending_tx_at_ms_{0};   // millis() target for next reply (0 = none)
  uint32_t last_tx_ms_{0};
  uint32_t last_rx_ms_{0};         // for alive watchdog
  bool     have_seen_rx_{false};

  // ---- RX ring buffer ----
  std::vector<uint8_t> rx_buf_;
  static constexpr size_t RX_BUF_MAX = 512;

  // ---- Timing constants ----
  static constexpr uint32_t RESPONSE_DELAY_MS  = 17;
  static constexpr uint32_t KEEPALIVE_MS        = 500;
  static constexpr uint32_t ALIVE_TIMEOUT_MS    = 3000;
  static constexpr uint32_t SYNC_DEBOUNCE_MS    = 2000;
};

}  // namespace pynuaire
}  // namespace esphome
