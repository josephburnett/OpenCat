# mind/firmware

Stored firmware images for `kilo flash`.

| File | What | Used by |
|---|---|---|
| `opencat-legacy.hex` | The legacy Petoi OpenCat firmware for Kilo (token protocol at 115200, uses the legacy EEPROM calibration). Built from `OpenCat.ino` on `mega` at 64b9447 (32016 B flash, 1620 B RAM; md5 b3e85cc029108f7a56b75c11a7626fde, byte-identical on x86-64 and the Pi). Rebuild with `arduino-cli compile --fqbn arduino:avr:uno --output-dir out .` and copy `out/OpenCat.ino.hex` here after any legacy change. | `kilo flash --legacy` (H15) |

The legacy image stays available until the new body firmware is accepted
(docs/architecture.md §7: `OpenCat.ino` and `src/` are deleted after P2, and
the hex kept here is the way back).

Flashing the legacy image does not touch EEPROM (avrdude runs with `-D` and
writes flash only), so it finds its calibration intact.
