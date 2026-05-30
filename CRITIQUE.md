# Code Review Findings — bms_reader.py

Multi-angle static review of the DALY BMS serial reader. Findings ranked by severity.

---

## Safety

### 1. Temperature scope silently drops thermal fault readings
**Line 117** — `hi=140` rejects raw values above 140 (= 100°C after the −40 offset). A pack at 105°C reports raw=145, fails the scope check, hits `continue`, and `BMS_Data['temperatures']` retains the last safe reading. A thermal runaway event is invisible to any consumer.

**Fix:** Raise `hi` to the physical maximum the BMS can report (e.g. 190 for 150°C), or remove the upper bound entirely and let consumers check converted values.

---

## Correctness

### 2. `battery_count=0` sends a zero-register Modbus request, desyncing the serial buffer
**Line 111** — If the BMS returns 0 for register 0x3C (plausible during init or fault), `n_cells=0`. `_build_request(0x00, 0)` emits an illegal Modbus frame. `ser.read(5)` then consumes the first 5 bytes of the *next* register's real response, misaligning every subsequent read in the poll cycle — a cascade of `None` returns and stale values for the remainder of the loop.

**Fix:** Guard `n_cells` and `n_temps` with a minimum of 1, or skip the read entirely when n=0.

### 3. `battery_count` is read *after* the registers that depend on it
**Line 116** — `_register_map()` reads `BMS_Data.get('battery_count', 48)` at call time, but `battery_count` (0x3C) appears later in the list than `cell_voltages` (0x00). On the first poll cycle the count defaults to 48. On a 16-cell BMS, 32 undefined registers are requested — the BMS may return junk that passes CRC, storing 32 phantom cell voltages.

**Fix:** Either move `battery_count` and `temp_sensor_count` to the front of the register list, or do a dedicated pre-poll read of those two registers before the main loop.

### 4. Tautological `(0, 65535)` scopes on ~30 scaled registers
**Lines 119–168** — `struct.unpack_from('>H', ...)` always returns 0–65535, so any scope of `(0, 65535)` or wider never rejects anything. Affected registers include:
- `soc`: raw=65535 → stored as 6553%
- `current`: raw=0 → stored as −3000 A
- All min/max voltage and temperature summary fields

Only `cell_voltages` (hi=5000) and `temperatures` (hi=140) have physically meaningful bounds. Several entries also use `hi=65536` or `hi=65537` — impossible values for a uint16 — indicating copy-paste drift.

**Fix:** Set meaningful upper bounds for each scaled register (e.g. `soc` hi=1000, `current` hi=60000 for ±3000 A range) or use `None` explicitly where no useful bound exists.

### 5. Modbus exception responses indistinguishable from cable fault
**Line 63** — If the BMS sends a Modbus exception frame (function byte = 0x83), the length mismatch or bad-header check returns `None` — the same path as a timeout or disconnected cable. There is no log message that distinguishes "device actively rejected the request" from "nothing arrived". Plausible under BMS protection events.

**Fix:** Check for `cmd == CMD_READ | 0x80` before the generic bad-header return and log the exception code.

---

## Reliability

### 6. `SerialException` propagates uncaught, crashing the daemon
**Line 196** — `ser.write()` and `ser.read()` in `_read_registers` have no `try/except`. A USB cable jiggle raises `serial.SerialException`, propagates through `_poll()` and the `while True` loop, and terminates the process. There is no reconnection or retry logic.

**Fix:** Wrap serial I/O in `_read_registers` with `try/except serial.SerialException`, and add an outer reconnect loop in `main()` with exponential backoff.

---

## Design

### 7. Failed reads leave stale values in `BMS_Data` with no freshness signal
**Line 179** — Every `continue` (on `None` or scope failure) leaves the previous value in place with no timestamp, `valid` flag, or expiry. `main()` prints `"BMS_Data updated — N keys"` unconditionally, implying freshness regardless of how many keys were actually refreshed.

**Fix:** Store `(value, timestamp)` tuples per key, or maintain a separate `BMS_Data_updated` dict of per-key timestamps so consumers can detect staleness.

---

## Efficiency

### 8. `time.sleep(POLL_INTERVAL)` is additive, not a fixed-rate mechanism
**Line 199** — The 1-second sleep runs *after* `_poll()` completes. With `TIMEOUT=2.0` and 50 register reads, a single timeout extends the cycle by 2 seconds; five timeouts yields an 11-second cycle.

**Fix:**
```python
next_tick = time.monotonic() + POLL_INTERVAL
_poll(ser)
time.sleep(max(0, next_tick - time.monotonic()))
```

### 9. ~43 individual Modbus requests for a contiguous register block
**Lines 124–167** — Registers 0x3E–0x73 (54 addresses with small gaps) are read one at a time. A single batch `READ 0x3E–0x73` collapses 42 round-trips into 1, plus 0x7E as a second request. At 9600 baud with the DALY's ~20ms response cadence, 42 unnecessary round-trips ≈ 840ms of wire time per cycle.

**Fix:** Read the block in one request, parse the full response array, and extract individual values by offset.

### 10. `_register_map()` rebuilds 50 tuples and ~45 lambda objects every poll cycle
**Line 110** — `battery_count` and `temp_sensor_count` are hardware constants that don't change after the first successful read. The full list and all its closures are allocated and GC'd every second.

**Fix:** Cache the register map after the first successful read of the two count registers, and rebuild only when they change.

---

## Summary

| # | Severity | Issue |
|---|---|---|
| 1 | Safety | Overtemp readings silently discarded |
| 2 | Correctness | Zero cell-count desyncs serial stream |
| 3 | Correctness | `battery_count` ordering — first cycle over-reads |
| 4 | Correctness | Tautological scopes pass impossible values |
| 5 | Correctness | Exception frames indistinguishable from cable fault |
| 6 | Reliability | `SerialException` crashes daemon |
| 7 | Design | Stale data with no freshness signal |
| 8 | Efficiency | Sleep additive with poll time |
| 9 | Efficiency | 43 round-trips instead of 2 batch reads |
| 10 | Efficiency | Register map rebuilt every cycle |
