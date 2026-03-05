# Clocktopus🐙🕰️🎛️
Clocktopus is a small, friendly device that lets your electronic instruments play together — in time, in sync, in conversation with each other.

If you've ever tried to get two or more electronic instruments, drum machines, sequencers, or synthesisers to lock into the same rhythm at the same time, you'll know the frustration: tempos drift, things fall out of sync.  By the time you've gotten everything synced up, inspiration is hampered.

Clocktopus solves this. Plug it in, connect your gear and press the big, glowing button — up to eight of you (or your band's) instruments undulate together to the tempo-locked pulsing glow.  Adjust the big, chunky tempo dial and your session's instruments speed up and slow down as one.

And because each of the 8 outputs is individually configurable - through an incredibly tight, chitin-solid clock driven by per-port hardware UART/GPIO  — each can run at its _own_ rhythmic subdivision, groove and swing via universal 3.5mm TRS MIDI OR 3.5mm CV/gate jacks for Eurorack and boutique instruments. 

Clocktopus also sends it's main clock over USB to your DAW  and - at the per-port configuration level - adjustable, high-resolution PPQ clock division ensuring far-reaching compatibility in your ecosystem of musical instruments.  No more double/half-speed sync issues across manufacturer devices.  

Additionally, a TRS Type A to DIN (5-pin) and TRS Type A to Type B are included, so your other instruments are not left out.


# Clocktopus — Feature List

---

## Connectivity

- **8 independent outputs** — each individually configurable as either MIDI
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
- **Sub-microsecond timing jitter** — hardware timer-driven clock engine
  for the tightest possible sync
- **MIDI clock output** — standard 24 PPQN on all MIDI-configured ports,
  compatible with virtually every MIDI device ever made
- **Start, Stop, and Continue** — full MIDI transport control sent to all
  connected devices simultaneously

---

## Rhythm & Feel

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
- **Half / Double speed switch** — instantly halves or doubles the effective
  tempo across all outputs simultaneously, without stopping the clock
- **Tap tempo** — set the tempo by feel, by tapping in time

---

## Live Performance

- **One-button Start / Stop** — large illuminated button for confident
  stage use
- **Double-press Resync** — press twice quickly to snap all outputs back
  to beat one without stopping playback; useful for correcting device-originating drift or
  re-locking after a transition
- **Instant half / double speed** — three-position toggle switch for
  real-time tempo doubling or halving mid-performance

---

## Visual Feedback

- **8 independent RGB LEDs** — one per output, each pulsing at that
  port's own clock rate so you can see the rhythmic structure of your
  whole setup at a glance
- **Colour-coded by output type** — blue glow for MIDI outputs, green
  glow for CV outputs, white flash on every clock pulse
- **OLED display** — shows current BPM, running/stopped status, selected
  port, and all per-port settings; three-level navigation menu for
  configuring each output

---

## Controls

- **Tempo dial** — large dedicated rotary dial for BPM adjustment;
  fine (0.5 BPM) and coarse (5 BPM) modes
- **Navigation encoder** — push-turn encoder for navigating and editing
  all per-port settings without menus getting in the way of playing
- **Three-level settings hierarchy** — select port → select parameter →
  edit value; long-press returns to the top level from anywhere

---

## Per-Port Settings (configurable per output)

- Output type: MIDI or CV
- Clock division / subdivision
- Swing amount
- Enable / disable

---

## Enclosure & Design

- **Faceted dome enclosure** — distinctive half-sphere form, stable on
  any surface, designed to be as wonderful to look at as to use
- **Open hardware and firmware** — the full design and code are openly
  available; Clocktopus can be understood, repaired, and modified by
  anyone with the curiosity to look inside

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

---

*Current version: Clocktopus v0.1 prototype*
*Platform: Teensy 4.1 — open hardware*
