# P0 hardware test: results log

Runs of docs/p0-hardware-test.md on Kilo. Newest first.

## 2026-10-10, from the Pi over the CH343 adapter

Setup: Pi 2 B (`kilo`), mind in `~/OpenCat/.venv`, `KILO_PORT=/dev/ttyACM0`.
Battery read 6.87 V before the first flash and dropped out (NO_BATTERY,
≈0.1 V) before the posture steps; the pack had tripped its cutoff and went
on the charger. The second half ran at 20:20 with the pack at 8.43 V.

| Step | Result |
|---|---|
| 0 setup | Pi venv recipe works. arduino-cli compile of body: 30 s. |
| 1 EEPROM dump | 283 bytes captured, committed as docs/kilo-eeprom-legacy.md. Legacy hex (32016 B, md5 b3e85cc0…) committed to mind/firmware/. |
| 2 host checks | Laptop: all green. Pi: C test links failed with ASan `__atomic_load_8` until `-latomic` was added (fixed in host_tests.sh / protocol/test.sh). |
| 3 flash | 18164 B, size_check PASS, `verified: HELLO reset_cause=FLASHED`. Owner saw: pose held, lay down, went limp (not fully certain). **config DEFAULTED** on the first build (see below). |
| 4 status | After the fix (build 6a5f7a1): `config OK`, pitch/roll level (4°/−3° lying limp), overruns 0, max tick 6.5 ms, stack free 749 B, dmp 100 Hz, i2c_err 0. EEPROM 0..282 byte-identical to the dump. Config slot A written at 704 with the six legacy MPU offsets. |
| 5 postures | First pass had no battery power (nothing moved). Second pass at 8.4 V: rest, stand, sit, balance, rest all executed; PCA read-back while STAND: ch0 1575, ch1 1170, ch4 1531, ch6 2200, ch7 1663, ch8 1663, ch9 1181, ch11 1356, ch12 1400, ch14 2145, ch15 1816 (ch2 "off": tail, 0 count read back). After every hold the passive timeline showed stop at +0.5 s and REST at +1.5 s; LIMP at +12.5 s on the 14 s watch. |
| 6 cal/params | `cal show` = the 16 legacy CALIB offsets exactly. 13 params at defaults; `param set WD_LIMP_S 5` / `get` round-trip ok. |
| 7 power cycle | Done implicitly when the charged pack was reconnected: `reset COLD`, `config OK`, mode LIMP, servos unpowered, uptime counting. |
| 8 watchdog | `kilo posture stand --for 2 --then-watch 15`: WATCHDOG event and IDLE→LINK_LOST at +0.5 s, REFLEX (REST) at +1.5 s, servos off (LIMP) at +12.5 s = 1.5 + 1 s move + WD_LIMP_S. Telemetry steady at 25 Hz, 0 frames sent while watching. |
| 9 rollback | `kilo flash --legacy`: 32016 B verified, legacy banner (Nybble / N_240907 / Ready!), `kup` and `krest` answered. P0 flashed back afterwards (build 6a383b8): it held the legacy REST pose on boot, reset cause COLD because the legacy firmware had overwritten the .noinit record. |

### Findings

1. **Legacy import rejected (fixed, fa33177).** `check_legacy()` failed on
   joint 2 (tail) direction: the firmware's joint table had −1, copied from
   upstream commit b152455 (Bittle arm support), while Kilo's EEPROM and the
   legacy firmware use +1. With DEFAULTED config the MPU offsets stayed at
   zero, which showed as pitch 64°/roll 13° while lying still. Kilo's exact
   dump is now a permanent host test (body/test/test_kilo_eeprom.cpp).
2. **Port default on the Pi (fixed, 9a15f91).** The header UART was chosen
   over the plugged-in USB adapter and GPIO4 was pulsed. A present USB adapter
   now wins.
3. **Resets on every command after a flash (fixed, 9a15f91).** avrdude turns
   HUPCL back on; the mind now clears it on every open and after avrdude. One
   residual: the first `kilo status` right after `kilo flash` still showed a
   fresh boot (WARM_UNEXPECTED, uptime 0); later commands did not reset. To
   be re-checked.
4. **`kilo flash -v` is wrong**: global flags go first, `kilo -v flash`
   (docs fixed).
5. **Battery**: 6.87 V read as 3 % by the mind's estimated curve and tripped
   BATT_WARN (6.9 V); legacy only warns below 6.5 V. The pack then cut out.
   Compare the reported mV with a meter once charged (owner input).
6. Link decode errors on connect: one `bad_cobs`/`bad_crc` per session from
   boot garbage. Harmless, counted.
7. `imu_stale` reached 3 over 153 s with DMP at 100 Hz. Watch in P1.
8. **Attitude follows handling.** Kilo was held in hand and tilted during
   the step 8 watch; the pitch 15.6° / roll −70.5° sample at +7.1 s was real.
   The owner reported the postures as correct. Still worth doing in P1: trip
   the fall reflex only on several consecutive samples over the threshold.
9. The watchdog cannot be observed with an active `kilo` command because the
   mind feeds it; `kilo monitor --passive` and `posture --then-watch` were
   added (239a89d). The body's mode stays LINK_LOST through all watchdog
   stages; REST shows as owners REFLEX/REFLEX and LIMP as `-servos_powered`.
10. LIMP comes 12.5 s after the link drops (1.5 s + 1 s REST move + 10 s);
    the vbody counts from 11.5 s. Align the vbody in P1.
