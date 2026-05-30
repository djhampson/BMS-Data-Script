# DALY Smart BMS Model K — UART Data Reader

Reads real-time data from a DALY Smart BMS (Model K) over UART/RS-485 using the Modbus protocol and maintains a Python dictionary (`BMS_Data`) of current values.

## Hardware

- DALY Smart BMS Model K
- Raspberry Pi 3B (Debian)
- USB-to-serial adapter connected to the BMS UART2 port

## Requirements

```
pyserial
```

```bash
pip install pyserial
```

## Configuration

Edit the constants at the top of `bms_reader.py`:

| Constant | Default | Description |
|---|---|---|
| `PORT` | `/dev/ttyUSB0` | Serial device (check with `ls /dev/ttyUSB*`) |
| `BAUD` | `9600` | Must match BMS setting |
| `TIMEOUT` | `2.0` | Seconds to wait for a response frame |
| `POLL_INTERVAL` | `1` | Seconds between full poll cycles |

## Usage

```bash
python3 bms_reader.py
```

The script opens the serial port, then repeatedly loops through all registers, updating `BMS_Data` after each complete cycle.

## Protocol

- **Serial**: 9600 baud, 8 data bits, 1 stop bit, no parity
- **Framing**: Modbus RTU (function code 0x03 — read holding registers)
- **Request slave address**: `0x81`
- **Response slave address**: `0x51`
- **CRC**: CRC-16 Modbus (poly `0xA001`)

### Request frame

| Byte | 0 | 1 | 2–3 | 4–5 | 6–7 |
|---|---|---|---|---|---|
| Field | ADDR (`0x81`) | CMD (`0x03`) | Register start address | Number of registers (N) | CRC-16 (LE) |

### Response frame

| Byte | 0 | 1 | 2 | 3 … (2N+2) | last 2 |
|---|---|---|---|---|---|
| Field | ADDR (`0x51`) | CMD (`0x03`) | Length (= 2N) | Register data | CRC-16 (LE) |

## BMS_Data dictionary

All values are converted to human-readable units:

| Key | Unit | Notes |
|---|---|---|
| `cell_voltages` | V (float list) | One entry per cell; list length = `battery_count` |
| `temperatures` | °C (int list) | One entry per sensor; list length = `temp_sensor_count` |
| `total_voltage` | V | Pack voltage |
| `current` | A | Positive = discharge, negative = charge |
| `soc` | % | State of charge |
| `life` | — | Heartbeat counter |
| `battery_count` | — | Number of cells in pack |
| `temp_sensor_count` | — | Number of temperature sensors |
| `max_cell_voltage` | mV | |
| `max_cell_voltage_num` | — | Cell index |
| `min_cell_voltage` | mV | |
| `min_cell_voltage_num` | — | Cell index |
| `cell_voltage_diff` | mV | Max − min |
| `max_cell_temp` | °C | |
| `max_cell_temp_num` | — | Sensor index |
| `min_cell_temp` | °C | |
| `min_cell_temp_num` | — | Sensor index |
| `cell_temp_diff` | °C | Max − min |
| `charge_discharge` | — | 0 = idle, 1 = charging, 2 = discharging |
| `charger_status` | — | 0 = not detected, 1 = detected |
| `load_status` | — | 0 = not detected, 1 = detected |
| `remaining_capacity` | Ah | |
| `cycle_count` | — | Charge cycles |
| `balance_state` | — | 0 = off, 1 = passive, 2 = active |
| `balance_positions` | bitmask | Bit N = cell N is balancing |
| `charge_mos` | — | 0 = off, 1 = on |
| `discharge_mos` | — | 0 = off, 1 = on |
| `precharge_mos` | — | 0 = off, 1 = on |
| `heating_mos` | — | 0 = off, 1 = on |
| `fan_mos` | — | 0 = off, 1 = on |
| `avg_cell_voltage` | mV | |
| `power` | W | |
| `energy` | Wh | |
| `mos_temp` | °C | |
| `ambient_temp` | °C | |
| `heating_temp` | °C | |
| `heating_current` | A | |
| `current_limit_state` | — | 0 = off, 1 = on |
| `current_limit` | A | Positive = discharge limit, negative = charge limit |
| `rtc` | string | `YYYY-MM-DD HH:MM:SS` |
| `remaining_charge_time` | min | |
| `di_do_status` | bitmask | Bits 0–7: DI1–DI8; bits 8–15: DO1–DO8 |
| `wake_source` | bitmask | bit0=key, bit1=button, bit2=485, bit3=CAN, bit4=current |
| `fault_0_1` … `fault_12_13` | bitmask | Raw fault registers — see protocol spec for bit definitions |
| `comm_interface_type` | — | 1 = RS-485, 2 = UART |

## Protocol reference

`KVMS_intranet_communication_UART_protocol_(customer_version).xlsx` (included in this repo's working directory, not tracked by git).
