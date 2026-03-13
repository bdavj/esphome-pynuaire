# esphome-pynuaire

ESPHome custom component for the **Nuaire Drimaster dMEV** continuous mechanical extract ventilation fan.

Tested on the **Drimaster ECO-HC**. Other Drimaster variants likely work — see [Contributing](#contributing).

## What it does

- Exposes the fan as a **Fan entity** in Home Assistant with 6 discrete speed levels (L1–L6)
- Reads the current speed from the motor and keeps HA in sync
- Uses the fan's internal UART bus directly — no extra hardware beyond an ESP32

## Hardware wiring

The Drimaster has a 4-pin header on its controller board carrying an inverted 57600 8N1 UART. Connect an ESP32 to this header:

| Fan header | ESP32 pin |
|-----------|-----------|
| TX (motor→ctrl) | RX pin (e.g. GPIO16) |
| RX (ctrl→motor) | TX pin (e.g. GPIO17) |
| GND | GND |
| 3.3 V | 3.3 V (or power ESP32 separately) |

> **Important:** The signal is electrically inverted. Set `inverted: true` on both pins in your YAML — see the example below.

## Installation

Add to your ESPHome config:

```yaml
external_components:
  - source: github://bdavj/esphome-pynuaire
    components: [pynuaire]
```

## Configuration

```yaml
uart:
  tx_pin:
    number: GPIO17
    inverted: true
  rx_pin:
    number: GPIO16
    inverted: true
  baud_rate: 57600
  id: nuaire_uart

fan:
  - platform: pynuaire
    uart_id: nuaire_uart
    name: "Drimaster Fan"
    default_level: 3   # L1–L6, applied on boot (default: 3)
```

See [`example.yaml`](example.yaml) for a full working config.

### Options

| Key | Default | Description |
|-----|---------|-------------|
| `uart_id` | required | ID of the UART component above |
| `name` | required | Entity name in Home Assistant |
| `default_level` | `3` | Speed level set on boot (1–6) |

## Speed levels

| Level | HA speed | Typical use |
|-------|----------|-------------|
| L1 | 17% | Minimum / standby |
| L2 | 33% | Low |
| L3 | 50% | Medium (recommended default) |
| L4 | 67% | High |
| L5 | 83% | Boost |
| L6 | 100% | Maximum boost |

## Protocol

The Drimaster uses a proprietary nibble-encoded UART protocol. Full documentation is in the companion [pinuaire](https://github.com/bdavj/pinuaire) repository, including a Python controller and Saleae capture analysis tools.

## Contributing

This component was developed against the **Drimaster ECO-HC**. If you have a different Drimaster model and it works (or doesn't), please open an issue or PR — Saleae/logic analyser captures of your unit are especially welcome and will help extend support.

PRs welcome for:
- Other Drimaster variants (ECO, Eco Plus, DC+, etc.)
- Additional ESPHome entities (runtime hours sensor, fault status, etc.)
- Bug fixes

## Disclaimer

This project is not affiliated with, endorsed by, or associated with Nuaire Ltd in any way. Nuaire is a trademark of its respective owner. This is independent, community-developed software based on reverse engineering of the fan's internal protocol.

## Licence

MIT
