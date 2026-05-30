#!/usr/bin/env python3
"""DALY Smart BMS Model K - UART Modbus real-time data reader."""

import serial
import struct
import time

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'   # adjust to match your USB-serial adapter
BAUD = 9600
TIMEOUT = 2.0           # seconds to wait for a response frame

REQ_ADDR  = 0x81        # slave address used in request frames
RESP_ADDR = 0x51        # slave address returned in response frames
CMD_READ  = 0x03        # Modbus function code: read holding registers

POLL_INTERVAL = 1       # seconds between full poll cycles

BMS_Data = {}


# ---------------------------------------------------------------------------
# Modbus helpers
# ---------------------------------------------------------------------------

def _crc16(data: bytes) -> int:
    """Modbus CRC-16 (polynomial 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _build_request(start: int, n: int) -> bytes:
    """Build an 8-byte Modbus read request frame."""
    pkt = bytes([REQ_ADDR, CMD_READ,
                 (start >> 8) & 0xFF, start & 0xFF,
                 (n >> 8) & 0xFF,     n & 0xFF])
    crc = _crc16(pkt)
    return pkt + bytes([crc & 0xFF, crc >> 8])


def _read_registers(ser: serial.Serial, start: int, n: int):
    """
    Send a read request and return a list of n unsigned-16 register values.
    Returns None if the response is missing, malformed, or fails CRC.
    """
    ser.reset_input_buffer()
    ser.write(_build_request(start, n))

    expected = 5 + n * 2   # ADDR + CMD + LEN + (2*n data bytes) + 2 CRC bytes
    raw = ser.read(expected)

    if len(raw) < expected:
        print(f"[0x{start:02X}] Short response: {len(raw)}/{expected} bytes")
        return None

    addr, cmd, length = raw[0], raw[1], raw[2]
    if addr != RESP_ADDR or cmd != CMD_READ:
        print(f"[0x{start:02X}] Bad header: addr=0x{addr:02X} cmd=0x{cmd:02X}")
        return None
    if length != n * 2:
        print(f"[0x{start:02X}] Length mismatch: got {length}, expected {n * 2}")
        return None

    crc_recv = struct.unpack_from('<H', raw, 3 + n * 2)[0]
    crc_calc = _crc16(raw[:3 + n * 2])
    if crc_recv != crc_calc:
        print(f"[0x{start:02X}] CRC error: recv=0x{crc_recv:04X} calc=0x{crc_calc:04X}")
        return None

    return [struct.unpack_from('>H', raw, 3 + i * 2)[0] for i in range(n)]


# ---------------------------------------------------------------------------
# Converters
# ---------------------------------------------------------------------------

def _scope_ok(regs: list, lo, hi) -> bool:
    """Check that every raw register value is within [lo, hi]."""
    return all(lo <= v <= hi for v in regs)


def _rtc(regs: list) -> str:
    """
    Three registers encode: year/month | day/hour | minute/second.
    Example: 0x14080F081E38 → 2020-08-15 08:30:56
    """
    y, mo = regs[0] >> 8, regs[0] & 0xFF
    d,  h = regs[1] >> 8, regs[1] & 0xFF
    mi, s = regs[2] >> 8, regs[2] & 0xFF
    return f"20{y:02d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}"


# ---------------------------------------------------------------------------
# Register map
# ---------------------------------------------------------------------------
# Each tuple: (key, start_addr, num_registers, converter, scope_lo, scope_hi)
#
# converter : list[int] → value stored in BMS_Data
# scope     : raw register values must satisfy lo <= v <= hi (None = skip check)
#
# Cell voltages and temperatures use the battery/sensor count already in
# BMS_Data so subsequent cycles read only the populated registers.

def _register_map():
    n_cells = min(int(BMS_Data.get('battery_count',    48)), 48)
    n_temps  = min(int(BMS_Data.get('temp_sensor_count', 8)),  8)

    return [
        # key                      start  n         converter                              lo    hi
        ('cell_voltages',          0x00, n_cells,   lambda r: [v * 0.001 for v in r],     0,  5000),
        ('temperatures',           0x30, n_temps,   lambda r: [v - 40 for v in r],        0,   140),
        ('total_voltage',          0x38,  1,         lambda r: r[0] * 0.1,                None, None),
        ('current',                0x39,  1,         lambda r: (r[0] - 30000) * 0.1,      0, 65535),
        ('soc',                    0x3A,  1,         lambda r: r[0] * 0.1,                0, 65535),
        ('life',                   0x3B,  1,         lambda r: r[0],                      0, 65535),
        ('battery_count',          0x3C,  1,         lambda r: r[0],                      None, None),
        ('temp_sensor_count',      0x3D,  1,         lambda r: r[0],                      None, None),
        ('max_cell_voltage',       0x3E,  1,         lambda r: r[0],                      0, 65535),
        ('max_cell_voltage_num',   0x3F,  1,         lambda r: r[0],                      0, 65535),
        ('min_cell_voltage',       0x40,  1,         lambda r: r[0],                      0, 65535),
        ('min_cell_voltage_num',   0x41,  1,         lambda r: r[0],                      0, 65535),
        ('cell_voltage_diff',      0x42,  1,         lambda r: r[0],                      0, 65535),
        ('max_cell_temp',          0x43,  1,         lambda r: r[0] - 40,                 0, 65535),
        ('max_cell_temp_num',      0x44,  1,         lambda r: r[0],                      0, 65535),
        ('min_cell_temp',          0x45,  1,         lambda r: r[0] - 40,                 0, 65535),
        ('min_cell_temp_num',      0x46,  1,         lambda r: r[0],                      0, 65535),
        ('cell_temp_diff',         0x47,  1,         lambda r: r[0],                      0, 65535),
        ('charge_discharge',       0x48,  1,         lambda r: r[0],                      0, 65535),
        ('charger_status',         0x49,  1,         lambda r: r[0],                      0, 65535),
        ('load_status',            0x4A,  1,         lambda r: r[0],                      0, 65535),
        ('remaining_capacity',     0x4B,  1,         lambda r: r[0] * 0.1,                None, None),
        ('cycle_count',            0x4C,  1,         lambda r: r[0],                      None, None),
        ('balance_state',          0x4D,  1,         lambda r: r[0],                      0, 65535),
        # 3 registers → 48 bits, one per cell
        ('balance_positions',      0x4F,  3,
            lambda r: (r[0] << 32) | (r[1] << 16) | r[2],                                0, 65535),
        ('charge_mos',             0x52,  1,         lambda r: r[0],                      0, 65535),
        ('discharge_mos',          0x53,  1,         lambda r: r[0],                      0, 65535),
        ('precharge_mos',          0x54,  1,         lambda r: r[0],                      0, 65535),
        ('heating_mos',            0x55,  1,         lambda r: r[0],                      0, 65535),
        ('fan_mos',                0x56,  1,         lambda r: r[0],                      0, 65535),
        ('avg_cell_voltage',       0x57,  1,         lambda r: r[0],                      0, 65535),
        ('power',                  0x58,  1,         lambda r: r[0],                      0, 65535),
        ('energy',                 0x59,  1,         lambda r: r[0],                      0, 65535),
        ('mos_temp',               0x5A,  1,         lambda r: r[0] - 40,                 0, 65535),
        ('ambient_temp',           0x5B,  1,         lambda r: r[0] - 40,                 0, 65535),
        ('heating_temp',           0x5C,  1,         lambda r: r[0] - 40,                 0, 65535),
        ('heating_current',        0x5D,  1,         lambda r: r[0],                      0, 65535),
        ('current_limit_state',    0x5F,  1,         lambda r: r[0],                      0, 65536),
        ('current_limit',          0x60,  1,         lambda r: (r[0] - 30000) * 0.1,      0, 65536),
        ('rtc',                    0x61,  3,         _rtc,                                None, None),
        ('remaining_charge_time',  0x64,  1,         lambda r: r[0],                      0, 65536),
        ('di_do_status',           0x65,  1,         lambda r: r[0],                      0, 65535),
        ('wake_source',            0x6B,  1,         lambda r: r[0],                      0, 65536),
        ('fault_0_1',              0x6D,  1,         lambda r: r[0],                      0, 65536),
        ('fault_2_3',              0x6E,  1,         lambda r: r[0],                      0, 65536),
        ('fault_4_5',              0x6F,  1,         lambda r: r[0],                      0, 65536),
        ('fault_6_7',              0x70,  1,         lambda r: r[0],                      0, 65536),
        ('fault_8_9',              0x71,  1,         lambda r: r[0],                      0, 65536),
        ('fault_10_11',            0x72,  1,         lambda r: r[0],                      0, 65536),
        ('fault_12_13',            0x73,  1,         lambda r: r[0],                      0, 65536),
        ('comm_interface_type',    0x7E,  1,         lambda r: r[0],                      0, 65537),
    ]


# ---------------------------------------------------------------------------
# Poll loop
# ---------------------------------------------------------------------------

def _poll(ser: serial.Serial) -> None:
    for key, start, n, convert, lo, hi in _register_map():
        regs = _read_registers(ser, start, n)
        if regs is None:
            continue

        if lo is not None and not _scope_ok(regs, lo, hi):
            print(f"[0x{start:02X}] Scope error for '{key}': raw={regs}")
            continue

        try:
            BMS_Data[key] = convert(regs)
        except Exception as exc:
            print(f"[0x{start:02X}] Conversion error for '{key}': {exc}")


def main():
    with serial.Serial(PORT, BAUD, bytesize=8, stopbits=1,
                       parity=serial.PARITY_NONE, timeout=TIMEOUT) as ser:
        print(f"Opened {PORT} at {BAUD} baud. Starting poll loop.")
        while True:
            _poll(ser)
            print(f"BMS_Data updated — {len(BMS_Data)} keys")
            time.sleep(POLL_INTERVAL)


if __name__ == '__main__':
    main()
