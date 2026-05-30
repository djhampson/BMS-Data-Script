# DALY Smart BMS Model K — UART Data Reader

Reads real-time data from a DALY Smart BMS (Model K) over UART/RS-485 using the Modbus protocol and maintains a Python dictionary (`BMS_Data`) of current values.

The reader polls the BMS on a fixed cadence using batched Modbus reads, validates each value against its physical scope, records a per-key freshness timestamp, and automatically reconnects if the serial link drops.

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
| `POLL_INTERVAL` | `1.0` | Target seconds per full poll cycle |
| `RECONNECT_DELAY` | `5` | Seconds between reconnect attempts after a serial error |

## Usage

```bash
python3 bms_reader.py
```

The script opens the serial port, then repeatedly polls the BMS, updating `BMS_Data` after each complete cycle. The cycle period is held stable using a monotonic clock (it sleeps only the remainder of `POLL_INTERVAL`), so slow reads or timeouts don't compound the loop interval.

### Resilience

- **Auto-reconnect** — a `serial.SerialException` (e.g. an unplugged USB adapter) is caught, logged, and the port is reopened after `RECONNECT_DELAY` seconds instead of crashing the process.
- **Scope validation** — each value is checked against its physical range before being stored. Out-of-range readings are logged and the previous value is left in place, except temperatures, whose upper bound (150 °C) is wide enough that thermal-fault readings are never silently dropped.
- **Modbus exception detection** — exception frames (function byte `0x83`) are recognised and logged with their exception code, distinct from a cable fault or timeout.

### Data freshness

Failed or out-of-range reads leave the last good value in `BMS_Data`. To detect stale data, every successful write also records a timestamp. Use `is_fresh()` to check a key:

```python
from bms_reader import BMS_Data, is_fresh

if is_fresh('total_voltage'):          # updated within the last 5 s (default)
    print(BMS_Data['total_voltage'])

is_fresh('soc', max_age=2.0)           # custom freshness window in seconds
```

## Tests

A test suite (`test_bms_reader.py`, 67 tests) covers the CRC, frame building/parsing, scope validation, RTC decoding, freshness tracking, and reconnect behaviour. It uses a fake serial port and requires no hardware:

```bash
pip install pytest
pytest test_bms_reader.py
```

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

### Batched reads

Rather than one request per register, the poll cycle issues a handful of batched Modbus reads:

| Block | Registers | Contents |
|---|---|---|
| Bootstrap | `0x3C`–`0x3D` | `battery_count`, `temp_sensor_count` (read first to size the reads below) |
| 1 | `0x00` … | Cell voltages (`battery_count` registers) |
| 2 | `0x30` … | Temperatures (`temp_sensor_count` registers) |
| 3 | `0x38`–`0x65` | Main scalar data (46 registers; `0x4E` and `0x5E` are gaps) |
| 4 | `0x6B` | Wake source |
| 5 | `0x6D`–`0x73` | Fault codes |
| 6 | `0x7E` | Communication interface type |

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

`KVMS_intranet_communication_UART_protocol_(customer_version)..xlsx` — included in the repo. The `Real-time data` sheet defines the register addresses, content names, scopes, and data-calculation formulas; the `modbus format` sheet documents the frame layout.
