# Clocktopus🐙🕰️🎛️
Clocktopus is a small, friendly device that allows up to 8 electronic instruments to play together — in time and in sync.

If you've ever tried to get two or more electronic instruments, drum machines, sequencers, or synthesisers to lock into the same rhythm at the same time, you'll know the frustration: mismatched sync rates, incompatible ports or even tempo drifts.  By the time you've found some kind of compromise amidst troubleshooting your sync issues, you're out of breath, your inspiration ensnared.

Clocktopus solves this. Plug it in, connect your gear and press the glowing start button.  Up to 8 of you (or your band's) instruments undulate together to the tempo-locked, pulsing glow. Adjust the big, chunky tempo dial and your session's instruments speed up and slow down, breathing as one.

At the heart of Clocktopus lies a precise, high-resolution clock timer that sends stable, jitter-free sync signals to its 8 ports. Each port can be independently configured to send time-shifted, rhythmic subdivisions (with swing) to either a 3.5mm TRS MIDI or CV/gate jack output, ensuring seamless sync compatibility (and playability!) across various MIDI, Eurorack, and boutique instruments.

Finally, with it's open-source firmware, you can fine-tune Clocktopus's existing features or add entirely new ones: think tempo-synced, CV LFO output or odd time signatures per port. Or perhaps you'd want to add a different clock or polyrhythmic system based on West African djembe cycles, Indian tala systems, or Indonesian gamelan. Musical curiousity is encouraged.

Clocktopus soothes the stress of syncing your gear so you can play with other people more easily.


# Clocktopus — Feature List

---

## Connectivity

- **8 independent clock outputs** — each individually configurable as either MIDI
  or CV/gate clock
- **MIDI outputs** via 3.5mm TRS Type A jacks — the current standard across
  modern hardware synthesisers, sequencers, and drum machines
- **Eurorack CV/gate outputs** — 5V pulse clock compatible with modular
  synthesiser systems
- **Included adapters** — TRS Type A to DIN (5-pin) and TRS Type A to
  Type B, so Clocktopus works with everything you already own
- **USB connection** — powers the device and doubles as a USB MIDI output
  to your computer or DAW

---

## Timing & Sync

- **Tempo range** — 20 to 300 BPM
- **Internal high-resolution clock** — 96 pulses per quarter note internal
  resolution for precise, stable timing across all outputs
- **Low-jitter clock engine** — timer-driven core clock designed for
  sub-microsecond jitter (see prototype status below)
- **MIDI clock output** — standard 24 PPQN on all MIDI-configured ports,
  compatible with virtually every MIDI device ever made
- **Start, Stop, and Continue** — full MIDI transport control sent to all
  connected devices simultaneously

---

## Rhythm & Feel

- **Per-port time shift** — ±50 milliseconds in 0.5ms steps, to nudge an
  instrument earlier or later while everything else stays locked to the grid
- **Per-port clock division** — each output independently set to its own
  rhythmic subdivision:
  - Whole note (1 pulse per bar)
  - Half note
  - Quarter note
  - 8th note
  - 16th note
  - 32nd note
  - MIDI rate (24 PPQN raw)
  - Volca preset (Korg Volca-compatible sync pulse rate)
- **Per-port swing** — independently adjustable groove offset per output,
  from straight (50%) to heavy swing (90%), so individual instruments can
  feel different while staying locked together
- **Tap tempo** — set the tempo by feel, by tapping in time

---

## Live Performance

- **One-button Start / Stop** — large illuminated button for confident
  stage use
- **Double-press Resync** — tap twice quickly to snap all outputs back
  to beat one without stopping playback; useful for correcting device-originating drift or
  re-locking after a transition

---

## Visual Feedback

- **Colour-coded Start / Stop Button** — red for clock stopped, green for clock running
- **8 independent RGB LEDs** — one per output, each pulsing at that
  port's own clock rate so you can see the rhythmic structure of your
  whole setup at a glance
- **Colour-coded by output type** — blue glow for MIDI outputs, green
  glow for CV outputs, white flash for main clock pulse
- **OLED display** — shows current BPM, selected port and all per-port
  settings; three-level navigation menu for configuring each output. Transport
  state needs no words: the BPM readout blinks when the clock is stopped and
  holds steady when it runs

---

## Controls

- **Tempo dial** — large dedicated rotary dial for BPM adjustment;
  1 BPM per step, or 0.1 BPM while the navigation encoder is held
- **Navigation encoder** — push-turn encoder for navigating and editing
  all per-port settings without menus getting in the way of playing
- **Three-level settings hierarchy** — select port → select parameter →
  edit value; long-press returns to the top level from anywhere below the
  overview, where it mutes the selected port instead
- **Port mute** — long-press on a port in the overview mutes or unmutes it.
  Muted ports blink on the display, and a muted MIDI port is sent STOP so its
  follower parks cleanly rather than hanging on a clock that stopped arriving

---

## Per-Port Settings (configurable per output)

- Output type: MIDI or CV
- Time shift
- Clock division / subdivision
- Swing amount

Mute is not in this list — it is a top-level gesture on the overview, not a
menu item.

---

## Enclosure & Design

- **Faceted dome enclosure** — distinctive half-sphere form, stable on
  any surface, designed to be as wonderful to look at as to use
- **Open hardware and firmware** — the full design and code are openly
  available under the GNU GPL 3.0 (see LICENSE); Clocktopus can be
  understood, repaired, and modified by anyone with the curiosity to
  look inside

---

## Compatibility

Works with any device that accepts:
- MIDI clock via 3.5mm TRS (Type A or Type B via included adapter)
- MIDI clock via 5-pin DIN (via included adapter)
- Eurorack CV/gate clock (5V pulse)

Including but not limited to: synthesisers, drum machines, sequencers,
samplers, arpeggiators, Eurorack modular systems, DAWs (via USB MIDI),
and Korg Volca series (via Volca preset).

---

## In the Box

- Clocktopus device
- TRS Type A to 5-pin DIN adapter cable
- TRS Type A to TRS Type B adapter cable
- USB cable
- Manual

---

## Prototype Status

This README describes the finished Clocktopus. The firmware is at v0.11,
running on a breadboard prototype: a Teensy 4.1 with the 1.3" OLED, the
navigation and tempo encoders, and one TRS MIDI output attached, powered over
USB. As of 29 August 2026 it delivers a steady 24 PPQN USB MIDI clock with
Start/Stop/Resync transport, drives the display, syncs Ableton Live as a
follower over USB MIDI, clocks an external synth from MIDI port 1, navigates
its own menus, and dials tempo from the encoder. The NeoPixels, CV outputs and
MIDI ports 2-8 are not yet wired and remain untested on hardware. v0.7
reassigns pins so that all eight MIDI ports can transmit, but only port 1 has
ever had a jack on it. A
few listed features are still in development: MIDI clock swing, tap tempo, and
the hardware-timer pulse scheduler that will deliver the final jitter target
on shifted and swung outputs.

---

*Firmware: v0.11 — Platform: Teensy 4.1 — open hardware*
