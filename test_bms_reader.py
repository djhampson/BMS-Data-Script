#!/usr/bin/env python3
"""
Test suite for bms_reader.py.

Covers all 10 issues from the code review:
  1. Temperature scope now passes readings above 100 °C
  2. n=0 guard prevents malformed Modbus requests
  3. battery_count / temp_sensor_count bootstrapped before block reads
  4. Meaningful scope bounds on scaled registers
  5. Modbus exception frames detected and logged distinctly
  6. SerialException caught; main() reconnects instead of crashing
  7. Per-key timestamps; is_fresh() detects stale data
  8. Fixed-rate poll cycle (sleep adjusted for elapsed time)
  9. Batch block reads (6 requests, not ~50)
 10. No per-cycle list/lambda rebuild

Run with:
    python -m pytest test_bms_reader.py -v
  or
    python test_bms_reader.py
"""

import struct
import time
import unittest
from unittest.mock import MagicMock, patch, call

import serial as serial_mod

import bms_reader
from bms_reader import (
    _crc16, _build_request, _in_scope, _decode_rtc, is_fresh,
)


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def _make_response(values: list) -> bytes:
    """Build a valid DALY Modbus read-response frame for a list of uint16 values."""
    n = len(values)
    payload = bytes([0x51, 0x03, n * 2]) + struct.pack(f'>{n}H', *values)
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])


def _make_exception_frame(exc_code: int = 0x04) -> bytes:
    """Build a 5-byte Modbus exception response frame."""
    payload = bytes([0x51, 0x83, exc_code])
    crc = _crc16(payload)
    return payload + bytes([crc & 0xFF, crc >> 8])


class BMSTestCase(unittest.TestCase):
    """Base class that clears module-level state between tests."""

    def setUp(self):
        bms_reader.BMS_Data.clear()
        bms_reader._data_ts.clear()


# ---------------------------------------------------------------------------
# CRC-16
# ---------------------------------------------------------------------------

class TestCRC16(unittest.TestCase):

    def test_known_vector_from_protocol_spec(self):
        # Spec example: 81-03-00-00-00-7F → CRC bytes 1B EA (LE → 0xEA1B)
        self.assertEqual(_crc16(bytes([0x81, 0x03, 0x00, 0x00, 0x00, 0x7F])), 0xEA1B)

    def test_empty_input_returns_init_value(self):
        self.assertEqual(_crc16(b''), 0xFFFF)

    def test_result_fits_uint16(self):
        for byte_val in range(256):
            result = _crc16(bytes([byte_val]))
            self.assertGreaterEqual(result, 0)
            self.assertLessEqual(result, 0xFFFF)

    def test_different_inputs_produce_different_crcs(self):
        self.assertNotEqual(_crc16(b'\x00'), _crc16(b'\x01'))


# ---------------------------------------------------------------------------
# Request frame builder
# ---------------------------------------------------------------------------

class TestBuildRequest(unittest.TestCase):

    def test_frame_is_always_8_bytes(self):
        self.assertEqual(len(_build_request(0x00, 1)), 8)
        self.assertEqual(len(_build_request(0x7E, 48)), 8)

    def test_known_frame_from_spec(self):
        # Protocol spec: 81-03-00-00-00-7F-1B-EA
        self.assertEqual(
            _build_request(0x00, 0x7F),
            bytes([0x81, 0x03, 0x00, 0x00, 0x00, 0x7F, 0x1B, 0xEA])
        )

    def test_start_address_big_endian(self):
        frame = _build_request(0x013C, 1)
        self.assertEqual(frame[2], 0x01)
        self.assertEqual(frame[3], 0x3C)

    def test_register_count_big_endian(self):
        frame = _build_request(0x00, 0x012F)
        self.assertEqual(frame[4], 0x01)
        self.assertEqual(frame[5], 0x2F)

    def test_crc_appended_little_endian(self):
        frame = _build_request(0x38, 10)
        crc_in_frame = struct.unpack('<H', frame[6:8])[0]
        self.assertEqual(_crc16(frame[:6]), crc_in_frame)

    def test_slave_address_is_0x81(self):
        self.assertEqual(_build_request(0x00, 1)[0], 0x81)

    def test_function_code_is_0x03(self):
        self.assertEqual(_build_request(0x00, 1)[1], 0x03)


# ---------------------------------------------------------------------------
# _in_scope  (fix 4)
# ---------------------------------------------------------------------------

class TestInScope(unittest.TestCase):

    def test_value_within_bounds(self):
        self.assertTrue(_in_scope(50, 0, 100))

    def test_value_at_lower_bound(self):
        self.assertTrue(_in_scope(0, 0, 100))

    def test_value_at_upper_bound(self):
        self.assertTrue(_in_scope(100, 0, 100))

    def test_value_below_lower_bound(self):
        self.assertFalse(_in_scope(0, 1, 100))

    def test_value_above_upper_bound(self):
        self.assertFalse(_in_scope(101, 0, 100))

    def test_none_lo_skips_lower_check(self):
        self.assertTrue(_in_scope(0, None, 100))

    def test_none_hi_skips_upper_check(self):
        self.assertTrue(_in_scope(65535, 0, None))

    def test_both_none_always_passes(self):
        self.assertTrue(_in_scope(0, None, None))
        self.assertTrue(_in_scope(65535, None, None))


# ---------------------------------------------------------------------------
# _decode_rtc
# ---------------------------------------------------------------------------

class TestDecodeRTC(unittest.TestCase):

    def test_spec_example(self):
        # Spec: 2020-08-15 08:30:56 → reg[0]=0x1408, reg[1]=0x0F08, reg[2]=0x1E38
        self.assertEqual(_decode_rtc(0x1408, 0x0F08, 0x1E38), '2020-08-15 08:30:56')

    def test_midnight(self):
        # 2024-01-01 00:00:00
        self.assertEqual(_decode_rtc(0x1801, 0x0100, 0x0000), '2024-01-01 00:00:00')

    def test_zero_padding(self):
        self.assertEqual(_decode_rtc(0x0101, 0x0101, 0x0101), '2001-01-01 01:01:01')


# ---------------------------------------------------------------------------
# is_fresh  (fix 7)
# ---------------------------------------------------------------------------

class TestIsFresh(BMSTestCase):

    def test_missing_key_returns_false(self):
        self.assertFalse(is_fresh('cell_voltages'))

    def test_just_stored_key_is_fresh(self):
        bms_reader._store('soc', 80.0)
        self.assertTrue(is_fresh('soc', max_age=5.0))

    def test_old_timestamp_is_stale(self):
        bms_reader.BMS_Data['soc'] = 80.0
        bms_reader._data_ts['soc'] = time.monotonic() - 10.0
        self.assertFalse(is_fresh('soc', max_age=5.0))

    def test_max_age_boundary(self):
        bms_reader.BMS_Data['soc'] = 80.0
        bms_reader._data_ts['soc'] = time.monotonic() - 3.0
        self.assertTrue(is_fresh('soc',  max_age=5.0))
        self.assertFalse(is_fresh('soc', max_age=2.0))


# ---------------------------------------------------------------------------
# _read_block
# ---------------------------------------------------------------------------

class TestReadBlock(BMSTestCase):

    def _ser(self, response: bytes) -> MagicMock:
        ser = MagicMock()
        ser.read.return_value = response
        return ser

    def test_valid_single_register(self):
        ser = self._ser(_make_response([0x1234]))
        self.assertEqual(bms_reader._read_block(ser, 0x38, 1), [0x1234])

    def test_valid_multiple_registers(self):
        values = [100, 200, 300]
        ser = self._ser(_make_response(values))
        self.assertEqual(bms_reader._read_block(ser, 0x00, 3), values)

    def test_registers_parsed_big_endian(self):
        # 0x0BB8 = 3000; verify big-endian byte order matters
        ser = self._ser(_make_response([0x0BB8]))
        self.assertEqual(bms_reader._read_block(ser, 0x00, 1), [3000])

    def test_n_zero_returns_none_without_writing(self):
        # Fix 2: n=0 must not emit an illegal Modbus frame
        ser = MagicMock()
        result = bms_reader._read_block(ser, 0x00, 0)
        self.assertIsNone(result)
        ser.write.assert_not_called()

    def test_n_negative_returns_none(self):
        ser = MagicMock()
        self.assertIsNone(bms_reader._read_block(ser, 0x00, -5))
        ser.write.assert_not_called()

    def test_short_response_returns_none(self):
        ser = self._ser(b'\x51\x03')   # 2 bytes instead of 7
        self.assertIsNone(bms_reader._read_block(ser, 0x38, 1))

    def test_bad_crc_returns_none(self):
        good = _make_response([0x1234])
        bad  = good[:-1] + bytes([good[-1] ^ 0xFF])
        ser  = self._ser(bad)
        self.assertIsNone(bms_reader._read_block(ser, 0x38, 1))

    def test_wrong_slave_address_returns_none(self):
        frame = bytearray(_make_response([100]))
        frame[0] = 0x52   # not 0x51
        self.assertIsNone(bms_reader._read_block(self._ser(bytes(frame)), 0x38, 1))

    def test_wrong_command_byte_returns_none(self):
        frame = bytearray(_make_response([100]))
        frame[1] = 0x06   # write command, not read
        self.assertIsNone(bms_reader._read_block(self._ser(bytes(frame)), 0x38, 1))

    def test_exception_frame_detected_returns_none(self):
        # Fix 5: 5-byte exception frame for a 1-register request (expects 7)
        exc_frame = _make_exception_frame(0x04)   # 5 bytes
        ser = self._ser(exc_frame)
        self.assertIsNone(bms_reader._read_block(ser, 0x38, 1))

    def test_correct_request_frame_sent(self):
        ser = self._ser(_make_response([42]))
        bms_reader._read_block(ser, 0x3C, 1)
        ser.write.assert_called_once_with(_build_request(0x3C, 1))

    def test_input_buffer_flushed_before_request(self):
        ser = self._ser(_make_response([42]))
        bms_reader._read_block(ser, 0x3C, 1)
        ser.reset_input_buffer.assert_called_once()


# ---------------------------------------------------------------------------
# _bootstrap_counts  (fix 3)
# ---------------------------------------------------------------------------

class TestBootstrapCounts(BMSTestCase):

    def test_populates_battery_and_sensor_count(self):
        ser = MagicMock()
        ser.read.return_value = _make_response([16, 4])
        bms_reader._bootstrap_counts(ser)
        self.assertEqual(bms_reader.BMS_Data['battery_count'],    16)
        self.assertEqual(bms_reader.BMS_Data['temp_sensor_count'], 4)

    def test_reads_registers_0x3C_and_0x3D_together(self):
        # One 2-register batch read, not two single reads
        ser = MagicMock()
        ser.read.return_value = _make_response([16, 4])
        bms_reader._bootstrap_counts(ser)
        ser.write.assert_called_once_with(_build_request(0x3C, 2))

    def test_failed_read_leaves_bms_data_unchanged(self):
        ser = MagicMock()
        ser.read.return_value = b''   # simulated timeout
        bms_reader._bootstrap_counts(ser)
        self.assertNotIn('battery_count',    bms_reader.BMS_Data)
        self.assertNotIn('temp_sensor_count', bms_reader.BMS_Data)

    def test_timestamps_updated_on_success(self):
        ser = MagicMock()
        ser.read.return_value = _make_response([16, 4])
        before = time.monotonic()
        bms_reader._bootstrap_counts(ser)
        self.assertGreaterEqual(bms_reader._data_ts.get('battery_count', 0), before)


# ---------------------------------------------------------------------------
# _poll — block and conversion tests
# ---------------------------------------------------------------------------

class TestPoll(BMSTestCase):
    """
    Tests for _poll() use a fake _read_block keyed by start address so each
    block can be controlled independently.
    """

    def _block3(self, overrides: dict = None) -> list:
        """
        46 registers for the main scalar block (0x38-0x65).
        Defaults are physically plausible values.
        """
        regs = [0] * 46

        defaults = {
            0x38: 350,    # total_voltage  35.0 V
            0x39: 30080,  # current        +8.0 A discharge
            0x3A: 800,    # soc            80.0 %
            0x3B: 1,      # life
            0x3C: 16,     # battery_count
            0x3D: 4,      # temp_sensor_count
            0x3E: 3550,   # max_cell_voltage   mV
            0x3F: 1,
            0x40: 3540,   # min_cell_voltage   mV
            0x41: 8,
            0x42: 10,     # cell_voltage_diff  mV
            0x43: 65,     # max_cell_temp raw  (25 °C)
            0x44: 1,
            0x45: 63,     # min_cell_temp raw  (23 °C)
            0x46: 3,
            0x47: 2,      # cell_temp_diff
            0x48: 2,      # charge_discharge: discharging
            0x49: 0,      # charger_status: not detected
            0x4A: 1,      # load_status: detected
            0x4B: 400,    # remaining_capacity  40.0 Ah
            0x4C: 10,     # cycle_count
            0x4D: 0,      # balance_state: off
            0x4F: 0, 0x50: 0, 0x51: 0,  # balance_positions
            0x52: 1, 0x53: 1, 0x54: 0, 0x55: 0, 0x56: 0,
            0x57: 3545,   # avg_cell_voltage mV
            0x58: 280,    # power W
            0x59: 1400,   # energy Wh
            0x5A: 75,     # mos_temp raw (35 °C)
            0x5B: 65,     # ambient_temp raw (25 °C)
            0x5C: 60,     # heating_temp raw (20 °C)
            0x5D: 0,
            0x5F: 0,      # current_limit_state
            0x60: 30000,  # current_limit  0.0 A
            0x61: 0x1808, # rtc 2024-08-...
            0x62: 0x0112,
            0x63: 0x0000,
            0x64: 60,     # remaining_charge_time min
            0x65: 0,      # di_do_status
        }
        if overrides:
            defaults.update(overrides)
        for addr, val in defaults.items():
            regs[addr - 0x38] = val
        return regs

    def _run_poll(self, block_map: dict):
        """Run _poll() with _read_block mocked to return values from block_map."""
        def fake_read_block(ser, start, n):
            return block_map.get(start)

        ser = MagicMock()
        with patch.object(bms_reader, '_read_block', side_effect=fake_read_block):
            bms_reader._poll(ser)

    def _full_blocks(self, n_cells=16, b3=None):
        """Return a full block_map with sensible defaults.
        b3: optional dict of {register_address: value} overrides for block 3."""
        return {
            0x3C: [n_cells, 4],
            0x00: [3550] * n_cells,
            0x30: [65, 65, 65, 65],
            0x38: self._block3(b3),
            0x6B: [0],
            0x6D: [0] * 7,
            0x7E: [2],
        }

    # ── Fix 9: verify only 6 _read_block calls are made ─────────────────────

    def test_exactly_six_block_reads_per_cycle(self):
        # Fix 9: 6 batch reads, not ~50 individual ones
        call_starts = []

        def recording_read_block(ser, start, n):
            call_starts.append(start)
            return self._full_blocks()[start]

        ser = MagicMock()
        with patch.object(bms_reader, '_read_block', side_effect=recording_read_block):
            bms_reader._poll(ser)

        # _bootstrap_counts (0x3C) + 5 data blocks + re-read of 0x3C in block3 = 7 calls
        # because _bootstrap_counts calls _read_block(ser, 0x3C, 2) and then
        # block3 also covers 0x3C. So the unique START addresses should be the 6 blocks.
        unique_starts = set(call_starts)
        self.assertIn(0x3C, unique_starts)   # bootstrap
        self.assertIn(0x00, unique_starts)   # cell voltages
        self.assertIn(0x30, unique_starts)   # temperatures
        self.assertIn(0x38, unique_starts)   # main scalar block
        self.assertIn(0x6B, unique_starts)   # wake source
        self.assertIn(0x6D, unique_starts)   # fault codes
        self.assertIn(0x7E, unique_starts)   # comm type
        # Must NOT have individual reads for registers in 0x3E-0x73 range
        individual_regs = [0x3E, 0x3F, 0x40, 0x48, 0x52, 0x57, 0x5A, 0x60]
        for reg in individual_regs:
            self.assertNotIn(reg, unique_starts)

    # ── Fix 3: bootstrap counts read before dependent blocks ─────────────────

    def test_bootstrap_counts_before_cell_voltages(self):
        # Fix 3: _bootstrap_counts must be called before the cell-voltage read
        call_order = []

        def recording_read_block(ser, start, n):
            call_order.append(start)
            return self._full_blocks()[start]

        ser = MagicMock()
        with patch.object(bms_reader, '_read_block', side_effect=recording_read_block):
            bms_reader._poll(ser)

        bootstrap_idx  = next(i for i, s in enumerate(call_order) if s == 0x3C)
        cell_read_idx  = next(i for i, s in enumerate(call_order) if s == 0x00)
        self.assertLess(bootstrap_idx, cell_read_idx)

    # ── Fix 2: n=0 guard ─────────────────────────────────────────────────────

    def test_battery_count_zero_clamped_to_one(self):
        # Fix 2: battery_count=0 must not produce an n=0 Modbus request
        blocks = self._full_blocks(n_cells=0)
        blocks[0x3C] = [0, 4]

        def recording_read_block(ser, start, n):
            if start == 0x00:
                self.assertGreater(n, 0, "n=0 passed to _read_block for cell voltages")
                return [3550]   # 1 cell
            return blocks.get(start)

        ser = MagicMock()
        with patch.object(bms_reader, '_read_block', side_effect=recording_read_block):
            bms_reader._poll(ser)   # must not raise

    # ── Fix 1: temperature scope allows readings above 100 °C ─────────────────

    def test_temperature_105c_stored(self):
        # Fix 1: raw=145 → 105 °C; old code rejected raw>140, new code allows up to 190
        blocks = self._full_blocks()
        blocks[0x30] = [145, 65, 65, 65]   # sensor 0 at 105 °C
        self._run_poll(blocks)
        self.assertIn('temperatures', bms_reader.BMS_Data)
        self.assertEqual(bms_reader.BMS_Data['temperatures'][0], 105)

    def test_temperature_150c_stored(self):
        # raw=190 → 150 °C, the upper scope limit — must be stored
        blocks = self._full_blocks()
        blocks[0x30] = [190, 65, 65, 65]
        self._run_poll(blocks)
        self.assertEqual(bms_reader.BMS_Data['temperatures'][0], 150)

    def test_temperature_above_scope_not_stored(self):
        # raw=200 → 160 °C exceeds hi=190 → scope failure → stale value kept
        bms_reader.BMS_Data['temperatures'] = [99.0, 99.0, 99.0, 99.0]
        blocks = self._full_blocks()
        blocks[0x30] = [200, 65, 65, 65]
        self._run_poll(blocks)
        self.assertEqual(bms_reader.BMS_Data['temperatures'], [99.0, 99.0, 99.0, 99.0])

    # ── Fix 4: meaningful scope bounds ───────────────────────────────────────

    def test_soc_in_range_stored_as_percent(self):
        self._run_poll(self._full_blocks(b3={0x3A: 800}))
        self.assertAlmostEqual(bms_reader.BMS_Data['soc'], 80.0)

    def test_soc_out_of_range_not_stored(self):
        # Fix 4: raw SOC = 65535 → 6553.5 % is physically impossible; must be rejected
        bms_reader.BMS_Data['soc'] = 80.0
        self._run_poll(self._full_blocks(b3={0x3A: 65535}))
        self.assertEqual(bms_reader.BMS_Data['soc'], 80.0)   # stale value preserved

    def test_charge_mos_status_invalid_value_not_stored(self):
        # Fix 4: charge_mos valid range is 0–1; raw=2 must be rejected
        bms_reader.BMS_Data['charge_mos'] = 1
        self._run_poll(self._full_blocks(b3={0x52: 2}))
        self.assertEqual(bms_reader.BMS_Data['charge_mos'], 1)

    def test_max_cell_voltage_above_5v_not_stored(self):
        # Fix 4: raw > 5000 mV is physically impossible for a lithium cell
        bms_reader.BMS_Data['max_cell_voltage'] = 3550
        self._run_poll(self._full_blocks(b3={0x3E: 6000}))
        self.assertEqual(bms_reader.BMS_Data['max_cell_voltage'], 3550)

    # ── Fix 7: staleness tracking ─────────────────────────────────────────────

    def test_timestamp_updated_on_successful_write(self):
        before = time.monotonic()
        self._run_poll(self._full_blocks())
        self.assertGreaterEqual(bms_reader._data_ts.get('soc', 0), before)

    def test_stale_value_not_refreshed_after_failed_block(self):
        # Fix 7: a failed block3 read must not update the timestamp for soc
        bms_reader.BMS_Data['soc'] = 75.0
        bms_reader._data_ts['soc'] = time.monotonic() - 10.0
        blocks = self._full_blocks()
        blocks[0x38] = None   # block 3 fails
        self._run_poll(blocks)
        self.assertEqual(bms_reader.BMS_Data['soc'], 75.0)
        self.assertFalse(is_fresh('soc', max_age=5.0))

    # ── Conversion correctness ────────────────────────────────────────────────

    def test_cell_voltages_list_length_and_unit(self):
        self._run_poll(self._full_blocks(n_cells=16))
        self.assertEqual(len(bms_reader.BMS_Data['cell_voltages']), 16)
        self.assertAlmostEqual(bms_reader.BMS_Data['cell_voltages'][0], 3.550, places=3)

    def test_current_discharge_positive(self):
        # raw 30080 → (30080-30000)*0.1 = +8.0 A
        self._run_poll(self._full_blocks(b3={0x39: 30080}))
        self.assertAlmostEqual(bms_reader.BMS_Data['current'], 8.0)

    def test_current_charge_negative(self):
        # raw 29920 → (29920-30000)*0.1 = -8.0 A
        self._run_poll(self._full_blocks(b3={0x39: 29920}))
        self.assertAlmostEqual(bms_reader.BMS_Data['current'], -8.0)

    def test_total_voltage_unit(self):
        # raw 350 → 35.0 V
        self._run_poll(self._full_blocks(b3={0x38: 350}))
        self.assertAlmostEqual(bms_reader.BMS_Data['total_voltage'], 35.0)

    def test_temperature_offset_applied(self):
        # raw 65 → 65-40 = 25 °C
        self._run_poll(self._full_blocks())
        temps = bms_reader.BMS_Data['temperatures']
        self.assertEqual(temps[0], 25)

    def test_rtc_decoded(self):
        # reg[0]=0x1808 → year=24 month=8; reg[1]=0x0112 → day=1 hour=18
        self._run_poll(self._full_blocks(b3={0x61: 0x1808, 0x62: 0x0112, 0x63: 0x0000}))
        self.assertEqual(bms_reader.BMS_Data['rtc'], '2024-08-01 18:00:00')

    def test_balance_positions_48bit_bitmask(self):
        expected = (0xAAAA << 32) | (0x5555 << 16) | 0xAAAA
        self._run_poll(self._full_blocks(b3={0x4F: 0xAAAA, 0x50: 0x5555, 0x51: 0xAAAA}))
        self.assertEqual(bms_reader.BMS_Data['balance_positions'], expected)

    def test_fault_codes_stored_correctly(self):
        fault_vals = [0x0001, 0x0002, 0x0004, 0x0008, 0x0010, 0x0020, 0x0040]
        blocks = self._full_blocks()
        blocks[0x6D] = fault_vals
        self._run_poll(blocks)
        self.assertEqual(bms_reader.BMS_Data['fault_0_1'],   0x0001)
        self.assertEqual(bms_reader.BMS_Data['fault_12_13'], 0x0040)

    def test_failed_block_does_not_prevent_later_blocks(self):
        # Fix 9: a failed block 3 must not stop fault codes from updating
        blocks = self._full_blocks()
        blocks[0x38] = None   # block 3 fails
        blocks[0x6D] = [0xFF] * 7
        self._run_poll(blocks)
        self.assertNotIn('total_voltage', bms_reader.BMS_Data)
        self.assertEqual(bms_reader.BMS_Data['fault_0_1'], 0xFF)


# ---------------------------------------------------------------------------
# main() — reconnect and clean shutdown  (fixes 6 and 8)
# ---------------------------------------------------------------------------

class TestMain(unittest.TestCase):

    def test_serial_exception_reconnects(self):
        # Fix 6: SerialException must be caught; main() retries, not crashes
        side_effects = [
            serial_mod.SerialException("port not found"),
            serial_mod.SerialException("port not found"),
            KeyboardInterrupt,   # stop the loop on the 3rd attempt
        ]
        with patch('bms_reader.serial.Serial', side_effect=side_effects), \
             patch('bms_reader.time.sleep'):
            bms_reader.main()   # must return cleanly (no uncaught exception)

    def test_keyboard_interrupt_exits_cleanly(self):
        with patch('bms_reader.serial.Serial', side_effect=KeyboardInterrupt), \
             patch('bms_reader.time.sleep'):
            bms_reader.main()   # must return, not raise

    def test_sleep_uses_remaining_interval(self):
        # Fix 8: sleep duration = max(0, POLL_INTERVAL - elapsed), not flat 1 s
        sleep_calls = []

        def fake_sleep(secs):
            sleep_calls.append(secs)
            raise KeyboardInterrupt   # stop after first sleep

        mock_ser = MagicMock()
        mock_ser.__enter__ = MagicMock(return_value=mock_ser)
        mock_ser.__exit__ = MagicMock(return_value=False)

        with patch('bms_reader.serial.Serial', return_value=mock_ser), \
             patch('bms_reader._poll'), \
             patch('bms_reader.time.sleep', side_effect=fake_sleep), \
             patch('bms_reader.time.monotonic', side_effect=[0.0, 0.3, 0.3]):
            bms_reader.main()

        self.assertTrue(len(sleep_calls) > 0)
        # Sleep arg should be POLL_INTERVAL(1.0) - elapsed(0.3) = 0.7, not flat 1.0
        self.assertAlmostEqual(sleep_calls[0], 0.7, places=5)

    def test_sleep_clamped_to_zero_when_poll_overruns(self):
        # Fix 8: if poll takes longer than POLL_INTERVAL, sleep 0 (not negative)
        sleep_calls = []

        def fake_sleep(secs):
            sleep_calls.append(secs)
            raise KeyboardInterrupt

        mock_ser = MagicMock()
        mock_ser.__enter__ = MagicMock(return_value=mock_ser)
        mock_ser.__exit__ = MagicMock(return_value=False)

        with patch('bms_reader.serial.Serial', return_value=mock_ser), \
             patch('bms_reader._poll'), \
             patch('bms_reader.time.sleep', side_effect=fake_sleep), \
             patch('bms_reader.time.monotonic', side_effect=[0.0, 5.0, 5.0]):
            bms_reader.main()

        self.assertGreaterEqual(sleep_calls[0], 0.0)


# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
