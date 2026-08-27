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
Adafruit SH110X / GFX / NeoPixel. (The display is an SH1106 driven by
Adafruit_SH110X — SSD1306 is installed but unused.)

## Working rules

- Timing correctness is the product. Anything touching clockISR, the
  scheduled-pulse mechanism, or shared ISR/loop state needs the ISR-safety
  patterns already in the file: single-writer variables, signed-difference
  time comparisons (micros() wraps), no blocking calls at interrupt priority 0.
- Shift/swing are computed as offsets from the reference grid — never adjust a
  port's tick counter to move it in time.
- docs/ PDFs may lag the firmware (pin changes in v0.3: nav encoder 18/19→16/17,
  red button LED 26→28). Verify against the .ino header before trusting a pin
  number in the PDFs.
- Committed files never reveal what lives in `private/` — no filenames, no
  descriptions of contents. The most they may say is that the directory
  exists and is gitignored.
- README.md speaks for the finished instrument; its Prototype Status section
  and the "Known deferred work" list below describe the same gap from two
  sides. When a deferred item ships, update both in the same change.
- Don't flash any device unless asked. Say whether a flash is actually needed.
- **The CrowPanel belongs to Ableton Navigator, not Clocktopus.** It has been
  discussed as a possible companion display over UART, but nothing is built: no
  CrowPanel code or reference appears anywhere in this repo's tracked files.
  Never describe the CrowPanel as Clocktopus hardware.

## Values (full context in private/)

These shape code, docs, naming, and hardware decisions — not just marketing copy:

- **Language:** never "master"/"slave" (common in MIDI documentation). Use
  "leader/follower" for MIDI clock roles and "reference grid" for the internal
  timing grid. When quoting external docs that use the old terms, translate.
- **Tone:** playful and inclusive. The clock+octopus identity is core to the
  product, not decoration — the device is a tentacle moving in concert with
  the organism, not a "master" commanding followers.
- **Transcultural rhythm:** don't hard-code Western assumptions (4/4, straight
  subdivisions, 12-TET) as the only path. When a feature or UI label embeds a
  musical-cultural bias, flag it rather than silently assuming it.
- **Accessibility:** treat limited hand use or low vision as design inputs for
  hardware and UI decisions, not edge cases.
- **Openness:** GPL 3.0. Write code and docs so others can adapt Clocktopus to
  their own musical cultures — clarity over cleverness.
- **Supply chain:** when suggesting parts or BOM changes, flag known toxicity
  or ethical-sourcing concerns alongside price and availability.
- **Restraint:** the ethical context informs decisions but doesn't need to be
  foregrounded in product copy unless there's a compelling reason. Fun first.

## Known deferred work

- Hardware-timer scheduler for scheduled pulses (removes ~6ms worst-case
  jitter on shifted/swung pulses during OLED redraws; also fixes CV pulse-off
  stretching)
- WS2812Serial instead of Adafruit_NeoPixel (leds.show() disables interrupts
  ~240µs — the largest remaining jitter source)
- MIDI clock swing (needs per-port event queue — see v0.4 changelog)
- Tap tempo function exists but no button is wired to call it
- Hardware verified 2026-08-27 (v0.6, breadboard, USB powered): USB MIDI clock,
  Start/Stop/Resync, the 1.3" SH1106 OLED on pins 18/19 at 0x3C, Ableton Live
  following tempo over USB MIDI, MIDI port 1 clocking an external synth over
  TRS, and nav encoder navigation on pins 32/33 + button on 23.
- MIDI out needs no level shifter: direct 3.3V drive (33R to tip, 3.3V through
  10R to ring, GND to sleeve, TRS Type A) drives an opto-isolated input fine.
- Still unwired and untested: tempo encoder, NeoPixels, CV outs, MIDI ports 2-8.
  Remaining smoke test = shift on port 1 while watching the overview markers —
  now possible for the first time, since OLED + nav + port 1 all work.
- Two v0.5 timing fixes remain unproven and cannot be proven by DAW sync or a
  synth: tempo accuracy (120.000 vs 120.0077 BPM) needs an independent
  reference, since a follower tracks a wrong tempo as faithfully as a right
  one; the USB/DIN phase pre-load needs both streams timestamped side by side.
  Both are blocked on a USB-MIDI interface with a DIN input.
- The pin 20 / Serial5 TX conflict only affects MIDI port 5, not DIN wiring in
  general — port 1 is Serial1, TX on pin 1, no conflict.
- Use a real momentary button on pin 20, not a bare jumper: pulling a jumper
  chatters and toggles run/stop repeatedly. BTN_DEBOUNCE_MS cannot help.
