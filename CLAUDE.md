# Clocktopus

8-port MIDI/CV clock sync device on Teensy 4.1. Open hardware/firmware —
see README.md for the feature list this firmware implements.

## Layout

- `Clocktopus.ino` — the firmware. Single-file Arduino sketch; the changelog
  comment block at the top is the authoritative history of what changed and why.
- `docs/` — public build documentation (build guide, BOM, schematic, appendix).
- `private/` — gitignored. Never commit contents; not backed up by git.
- `archive/` — gitignored. Superseded v2/v3 sketches with known timing bugs.
  Reference only, do not flash.

## Compile check (no hardware needed)

    arduino-cli compile -b teensy:avr:teensy41:usb=serialmidi .

USB type "Serial + MIDI" is required — the firmware sends USB MIDI clock.
Installed toolchain: arduino-cli (Homebrew), teensy:avr core, libraries
Adafruit SSD1306 / GFX / NeoPixel.

## Working rules

- Timing correctness is the product. Anything touching clockISR, the
  scheduled-pulse mechanism, or shared ISR/loop state needs the ISR-safety
  patterns already in the file: single-writer variables, signed-difference
  time comparisons (micros() wraps), no blocking calls at interrupt priority 0.
- Shift/swing are computed as offsets from the master grid — never adjust a
  port's tick counter to move it in time.
- docs/ PDFs may lag the firmware (pin changes in v0.3: nav encoder 18/19→16/17,
  red button LED 26→28). Verify against the .ino header before trusting a pin
  number in the PDFs.

## Known deferred work

- Hardware-timer scheduler for scheduled pulses (removes ~6ms worst-case
  jitter on shifted/swung pulses during OLED redraws; also fixes CV pulse-off
  stretching)
- WS2812Serial instead of Adafruit_NeoPixel (leds.show() disables interrupts
  ~240µs — the largest remaining jitter source)
- MIDI clock swing (needs per-port event queue — see v0.4 changelog)
- Tap tempo function exists but no button is wired to call it
- Nothing has been flashed to hardware yet; smoke test = USB MIDI clock into
  a MIDI monitor, then shift on port 1 while watching the overview markers
