# Kilo

Kilo is a robot cat: a Petoi Nybble with a Raspberry Pi 2 B as its brain.
This repository is Kilo's software.

It began as a fork of Petoi's [OpenCat](https://github.com/PetoiCamp/OpenCat),
the stock firmware for the Nybble's NyBoard. The stock firmware does
everything on the board's small Arduino chip (an ATmega328P with 32 KB of
flash and 2 KB of RAM): postures, gaits, tricks, the remote, the sensors and
the personality. That left no room to grow, so we rewrote it as two minds
that share the work:

- `body/`: new firmware for the NyBoard, the lower mind. It does only what
  has to be fast and safe: moving the joints, balancing, walking, catching
  falls, watching the battery and the link to the Pi. If the Pi goes quiet,
  the body stops, lies down and goes limp on its own.
- `mind/`: a Python daemon on the Pi, the upper mind. It decides what Kilo
  does: postures, tricks, walking, the IR remote, perception and, later, a
  personality. It talks to the body over a serial link and offers a `kilo`
  command line.
- `protocol/`: the single definition of the messages between the two; the C
  and Python sides are generated from it.

Where to read more: `docs/architecture.md` is the design,
`docs/handoff-2026-10-11.md` is where things stand, `mind/README.md` lists the
commands, and `docs/p*-hardware-test*.md` record what has been tried on Kilo.

The original OpenCat firmware (`OpenCat.ino` and `src/`) is still in the
tree: `src/InstinctNybble.h` holds the stock postures and skills that the new
firmware and the trick importer read, and the tests check a few other headers
against it. The rest of the fork (its desktop UI, module tests, skill library)
was removed on 2026-10-11; git history has it. Licence: MIT, as OpenCat's.
