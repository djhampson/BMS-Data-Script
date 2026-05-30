#!/usr/bin/env python3
"""DALY Smart BMS Model K - UART Modbus real-time data reader."""

import serial
import struct
import time

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'       # adjust to match your USB-serial adapter
BAUD = 9600
TIMEOUT = 2.0               # seconds to wait for a response frame
POLL_INTERVAL = 1.0         # target seconds per full poll cycle
RECONNECT_DELAY = 5         # seconds between reconnect attempts

REQ_ADDR  = 0x81            # slave address in request frames
RESP_ADDR = 0x51            # slave address in response frames
CMD_READ  = 0x03            # Modbus function: read holding registers

# ---------------------------------------------------------------------------
# State
# Fix 7: _data_ts tracks when each key was last successfully written so
# consumers can detect stale readings.
# ---------------------------------------------------------------------------
BMS_Data = {}
_data_ts: dict[str, float] = {}


def is_fresh(key: str, max_age: float = 5.0) -> bool:
    """Return True if key was updated within max_age seconds."""
    ts = _data_ts.get(key)
    return ts is not None and (time.monotonic() - ts) <= max_age


# ---------------------------------------------------------------------------
# Modbus helpers
# ---------------------------------------------------------------------------

def _crc16(data: bytes) -> int:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _build_request(start: int, n: int) -> bytes:
    pkt = bytes([REQ_ADDR, CMD_READ,
                 (start >> 8) & 0xFF, start & 0xFF,
                 (n >> 8) & 0xFF,     n & 0xFF])
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, crc >> 8])


def _read_block(ser: serial.Serial, start: int, n: int) -> list[int] | None:
    """
    Send one Modbus read request and return n unsigned-16 register values.
    Returns None on any error (short response, bad header, CRC failure).

    Fix 2: guards n <= 0 (would send an illegal Modbus frame).
    Fix 5: distinguishes Modbus exception frames from plain comms failures.
    """
    if n <= 0:
        return None

    ser.reset_input_buffer()
    ser.write(_build_request(start, n))

    expected = 5 + n * 2   # ADDR + CMD + LEN + (2*n data bytes) + 2 CRC bytes
    raw = ser.read(expected)

    if len(raw) < expected:
        # Fix 5: recognise a 3-byte exception frame before logging "short response"
        if len(raw) >= 3 and raw[1] == CMD_READ | 0x80:
            print(f"[0x{start:02X}] BMS exception: code=0x{raw[2]:02X}")
        else:
            print(f"[0x{start:02X}] Short response: {len(raw)}/{expected} bytes")
        return None

    addr, cmd = raw[0], raw[1]
    if addr != RESP_ADDR:
        print(f"[0x{start:02X}] Unexpected slave address: 0x{addr:02X}")
        return None
    # Fix 5: exception in a full-length response (e.g. coincident length)
    if cmd == CMD_READ | 0x80:
        print(f"[0x{start:02X}] BMS exception: code=0x{raw[2]:02X}")
        return None
    if cmd != CMD_READ:
        print(f"[0x{start:02X}] Unexpected command byte: 0x{cmd:02X}")
        return None

    length = raw[2]
    if length != n * 2:
        print(f"[0x{start:02X}] Length mismatch: got {length}, expected {n * 2}")
        return None

    crc_recv = struct.unpack_from('<H', raw, 3 + n * 2)[0]
    if crc_recv != _crc16(raw[:3 + n * 2]):
        print(f"[0x{start:02X}] CRC error")
        return None

    return list(struct.unpack_from(f'>{n}H', raw, 3))


# ---------------------------------------------------------------------------
# Scope validation
# Fix 4: both bounds are independently optional (None = unbounded on that side).
# ---------------------------------------------------------------------------

def _in_scope(value: int, lo: int | None, hi: int | None) -> bool:
    return (lo is None or value >= lo) and (hi is None or value <= hi)


# ---------------------------------------------------------------------------
# Store helper
# ---------------------------------------------------------------------------

def _store(key: str, value) -> None:
    """Write to BMS_Data and record the update timestamp."""
    BMS_Data[key] = value
    _data_ts[key] = time.monotonic()


# ---------------------------------------------------------------------------
# RTC decoder
# ---------------------------------------------------------------------------

def _decode_rtc(r0: int, r1: int, r2: int) -> str:
    """Three registers → 'YYYY-MM-DD HH:MM:SS'."""
    y,  mo = r0 >> 8, r0 & 0xFF
    d,  h  = r1 >> 8, r1 & 0xFF
    mi, s  = r2 >> 8, r2 & 0xFF
    return f"20{y:02d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"


# ---------------------------------------------------------------------------
# Bootstrap: read cell/sensor counts before the main poll
# Fix 3: battery_count (0x3C) and temp_sensor_count (0x3D) must be populated
# before _poll() uses them to size the cell-voltage and temperature reads.
# ---------------------------------------------------------------------------

def _bootstrap_counts(ser: serial.Serial) -> None:
    regs = _read_block(ser, 0x3C, 2)
    if regs is not None:
        _store('battery_count',    regs[0])
        _store('temp_sensor_count', regs[1])


# ---------------------------------------------------------------------------
# Poll
# Fix 9: registers grouped into 6 batch reads instead of ~50 individual ones.
# Fix 10: no list/lambda rebuild per cycle — block sizes are computed once from
#         BMS_Data and the block reads are static.
# ---------------------------------------------------------------------------

def _poll(ser: serial.Serial) -> None:

    # Fix 3: counts first
    _bootstrap_counts(ser)

    # Fix 2, 10: safe cell/temp counts (min 1 avoids zero-register request)
    n_cells = max(1, min(int(BMS_Data.get('battery_count',    48)), 48))
    n_temps  = max(1, min(int(BMS_Data.get('temp_sensor_count', 8)),  8))

    # ── Block 1 ── Cell voltages  0x00 … 0x00+n_cells-1 ─────────────────────
    regs = _read_block(ser, 0x00, n_cells)
    if regs is not None:
        # Fix 1/4: scope 0–5000 mV per cell; flag but keep reading on partial failure
        ok = [_in_scope(v, 0, 5000) for v in regs]
        if all(ok):
            _store('cell_voltages', [v * 0.001 for v in regs])
        else:
            bad = [i for i, b in enumerate(ok) if not b]
            print(f"[0x00] Cell voltage(s) out of range at indices {bad}: {regs}")

    # ── Block 2 ── Temperatures  0x30 … 0x30+n_temps-1 ──────────────────────
    regs = _read_block(ser, 0x30, n_temps)
    if regs is not None:
        # Fix 1: hi=190 raw (= 150 °C) so thermal-runaway events are NOT dropped.
        # The original hi=140 (100 °C) silently discarded readings above 100 °C.
        ok = [_in_scope(v, 0, 190) for v in regs]
        if all(ok):
            _store('temperatures', [v - 40 for v in regs])
        else:
            bad = [i for i, b in enumerate(ok) if not b]
            print(f"[0x30] Temperature(s) out of range at indices {bad}: {regs}")

    # ── Block 3 ── Main scalar data  0x38 … 0x65  (46 registers) ────────────
    # Gaps within this range: 0x4E and 0x5E — read but ignored.
    regs = _read_block(ser, 0x38, 46)
    if regs is not None:

        def r(addr: int) -> int:
            return regs[addr - 0x38]

        def store_if(key, value, raw_val, lo=None, hi=None):
            if _in_scope(raw_val, lo, hi):
                _store(key, value)
            else:
                print(f"[0x{key}] {key} out of scope: raw={raw_val}")

        _store('total_voltage',  r(0x38) * 0.1)
        _store('current',        (r(0x39) - 30000) * 0.1)

        # Fix 4: SOC raw range 0–1000 (= 0.0 %–100.0 %)
        store_if('soc',          r(0x3A) * 0.1,   r(0x3A), 0, 1000)

        _store('life',           r(0x3B))
        _store('battery_count',  r(0x3C))
        _store('temp_sensor_count', r(0x3D))

        # Fix 4: voltage summaries scoped to 0–5000 mV
        store_if('max_cell_voltage',    r(0x3E), r(0x3E), 0, 5000)
        _store('max_cell_voltage_num',  r(0x3F))
        store_if('min_cell_voltage',    r(0x40), r(0x40), 0, 5000)
        _store('min_cell_voltage_num',  r(0x41))
        store_if('cell_voltage_diff',   r(0x42), r(0x42), 0, 5000)

        # Fix 4: temperature summaries raw 0–190 (= −40 °C … 150 °C)
        store_if('max_cell_temp',       r(0x43) - 40, r(0x43), 0, 190)
        _store('max_cell_temp_num',     r(0x44))
        store_if('min_cell_temp',       r(0x45) - 40, r(0x45), 0, 190)
        _store('min_cell_temp_num',     r(0x46))
        _store('cell_temp_diff',        r(0x47))

        # Fix 4: enum registers scoped to their documented values
        store_if('charge_discharge',    r(0x48), r(0x48), 0, 2)
        store_if('charger_status',      r(0x49), r(0x49), 0, 1)
        store_if('load_status',         r(0x4A), r(0x4A), 0, 1)
        _store('remaining_capacity',    r(0x4B) * 0.1)
        _store('cycle_count',           r(0x4C))
        store_if('balance_state',       r(0x4D), r(0x4D), 0, 2)
        # 0x4E — gap, not defined in protocol

        # balance_positions: 3 registers → 48-bit cell bitmask
        _store('balance_positions', (r(0x4F) << 32) | (r(0x50) << 16) | r(0x51))

        store_if('charge_mos',       r(0x52), r(0x52), 0, 1)
        store_if('discharge_mos',    r(0x53), r(0x53), 0, 1)
        store_if('precharge_mos',    r(0x54), r(0x54), 0, 1)
        store_if('heating_mos',      r(0x55), r(0x55), 0, 1)
        store_if('fan_mos',          r(0x56), r(0x56), 0, 1)

        store_if('avg_cell_voltage', r(0x57), r(0x57), 0, 5000)
        _store('power',              r(0x58))
        _store('energy',             r(0x59))

        # Fix 4: temperature summaries
        store_if('mos_temp',         r(0x5A) - 40, r(0x5A), 0, 190)
        store_if('ambient_temp',     r(0x5B) - 40, r(0x5B), 0, 190)
        store_if('heating_temp',     r(0x5C) - 40, r(0x5C), 0, 190)
        _store('heating_current',    r(0x5D))
        # 0x5E — gap, not defined in protocol

        store_if('current_limit_state', r(0x5F), r(0x5F), 0, 1)
        _store('current_limit',         (r(0x60) - 30000) * 0.1)
        _store('rtc',                   _decode_rtc(r(0x61), r(0x62), r(0x63)))
        _store('remaining_charge_time', r(0x64))
        _store('di_do_status',          r(0x65))

    # ── Block 4 ── Wake source  0x6B (isolated register) ────────────────────
    regs = _read_block(ser, 0x6B, 1)
    if regs is not None:
        _store('wake_source', regs[0])

    # ── Block 5 ── Fault codes  0x6D … 0x73  (7 registers) ─────────────────
    regs = _read_block(ser, 0x6D, 7)
    if regs is not None:
        for i, key in enumerate(['fault_0_1', 'fault_2_3', 'fault_4_5',
                                  'fault_6_7', 'fault_8_9', 'fault_10_11',
                                  'fault_12_13']):
            _store(key, regs[i])

    # ── Block 6 ── Communication interface type  0x7E ─────────────────────
    regs = _read_block(ser, 0x7E, 1)
    if regs is not None:
        _store('comm_interface_type', regs[0])


# ---------------------------------------------------------------------------
# Main loop
# Fix 6: SerialException is caught; the port is reopened after RECONNECT_DELAY.
# Fix 8: monotonic clock keeps the cycle period stable regardless of poll duration.
# ---------------------------------------------------------------------------

def main() -> None:
    while True:
        try:
            with serial.Serial(PORT, BAUD, bytesize=8, stopbits=1,
                               parity=serial.PARITY_NONE, timeout=TIMEOUT) as ser:
                print(f"Opened {PORT} at {BAUD} baud. Starting poll loop.")
                while True:
                    t_start = time.monotonic()

                    _poll(ser)

                    elapsed = time.monotonic() - t_start
                    print(f"BMS_Data updated — {len(BMS_Data)} keys ({elapsed:.2f}s)")

                    # Fix 8: sleep only the remainder of the target interval
                    time.sleep(max(0.0, POLL_INTERVAL - elapsed))

        except serial.SerialException as exc:
            # Fix 6: log and reconnect rather than crashing
            print(f"Serial error: {exc} — reconnecting in {RECONNECT_DELAY}s")
            time.sleep(RECONNECT_DELAY)

        except KeyboardInterrupt:
            print("Stopped.")
            break


if __name__ == '__main__':
    main()
