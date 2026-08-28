// ============================================================================
//  CLOCKTOPUS
//  8-Port MIDI / CV Eurorack Clock Sync Device
//  Platform: Teensy 4.1 (600MHz ARM Cortex-M7)
//
//  Hardware:
//    - 8x MIDI outputs via hardware UARTs (Serial1–Serial8)
//    - USB MIDI clock output (24 PPQN + Start/Stop/Continue) for DAW sync
//      (requires Tools > USB Type > Serial + MIDI)
//    - 8x Eurorack CV outputs via GPIO + 74AHCT125 level shifter
//    - 8x WS2812B NeoPixel LEDs (one per port)
//    - Encoder 1 (pins 30,31) — Tempo (non-detented / smooth)
//    - Encoder 2 (pins 32,33) — Port config navigation (detented with push button)
//    - Nav encoder button (pin 23)
//    - Start/Stop button (pin 20)
//    - Start/Stop button bicolour LED: green (pin 25, PWM), red (pin 28, PWM)
//      Frosted clear button cap. Green breathes to BPM when running.
//      Red glows dim steady when stopped.
//    - Speed switch HALF (pin 21), DOUBLE (pin 22)
//    - OLED display 1.3" 128x64 I2C (SH1106, address 0x3C)
//    - NeoPixel data pin 24
//
//  Dependencies (install via Arduino IDE Library Manager):
//    - Teensyduino (from pjrc.com — includes IntervalTimer, Encoder)
//    - Adafruit_SH110X
//    - Adafruit_GFX
//    - Adafruit_NeoPixel
// ============================================================================


// ============================================================================
//  CHANGELOG
//
//  v0.6  — 2026-08-27
//    FIXED   Both encoders were dead on every build, and always had been.
//            initMIDIPorts() calls begin(31250) on all 8 UARTs unconditionally
//            — it never checks ports[i].type — and it is the first call in
//            setup(). Serial3 owns pins 15/14 and Serial4 owns pins 16/17
//            (HardwareSerial3.cpp / HardwareSerial4.cpp in the core), and
//            HardwareSerial::begin() rewrites the pin mux for both RX and TX
//            regardless of whether anything is connected. The Encoder objects
//            are globals, so their INPUT_PULLUP setup ran before setup() and
//            was overwritten moments later. Symptom on hardware: the nav
//            encoder's push button works (pin 23, not a UART pin) while
//            rotation does nothing at all.
//            Encoders moved to pins no UART claims — tempo 14,15 → 30,31 and
//            nav 16,17 → 32,33. Chosen so they survive a full 8-port build
//            rather than needing a second remap later.
//            Found by hardware test, not by compiling: the sketch built clean
//            in this state from v0.1 onward.
//
//  v0.5  — 2026-07-21
//    Timing review against the E-RM jitter report and multiclock manual.
//
//    FIXED   USB MIDI clock and the DIN/CV outputs were permanently out of
//            phase. The ISR emits a USB tick when internalTickCount == 0
//            (read before the increment), so USB fired on ISR call 1 — but a
//            port only fires once portTickCount reaches its division, so a
//            MIDI port first fired on ISR call 4. That is a fixed 3-internal-
//            tick skew (15.6ms at 120 BPM) between the same device's two
//            clock outputs, for the whole session — not a startup transient.
//            startClock()/resyncClock() now pre-load portTickCount[] to
//            (division - 1) so every port fires on the first tick alongside
//            USB. What remains is a uniform ~1-internal-tick latency between
//            the START byte and the first clock, which moves all outputs
//            together and is a calibration constant rather than skew.
//
//    FIXED   No contact debounce on either button. A bouncing start/stop
//            switch produced extra falling edges well inside DOUBLE_PRESS_MS,
//            so a single press could register as a double press and fire
//            resyncClock() mid-performance. Both buttons now require
//            BTN_DEBOUNCE_MS of settled state before an edge is accepted,
//            and the state latch only advances on an accepted edge.
//
//    FIXED   micros() timestamp 0 was overloaded as the "nothing pending"
//            sentinel in pulseTriggerTime[] and cvPulseOffTime[]. `now + delay`
//            genuinely evaluates to 0 once per micros() wrap (~71.6 min),
//            which silently dropped a scheduled pulse or — worse — left a CV
//            pin latched HIGH until that port's next pulse. Timestamps that
//            land on 0 are nudged to 1µs.
//
//    FIXED   Systematic tempo error from integer-microsecond timer periods.
//            IntervalTimer's PIT is clocked at 24MHz (resolution 1/24 µs) and
//            cyclesFromPeriod() takes a different path per argument type: an
//            integer period truncates to (24*P - 1), a float period rounds to
//            (24.0f*P - 0.5f). Handing it a uint32 µs value discarded that
//            resolution — at 120 BPM the exact tick period is 5208.333µs, so
//            5208 loaded 124992 cycles rather than 125000. The device ran
//            120.0077 BPM instead of 120.000: 0.0064% fast, ≈19ms of
//            accumulated phase drift every 5 minutes against a DAW at the same
//            nominal tempo. The period is now kept as a float for the timer
//            (pulseIntervalMicrosF) alongside the rounded integer copy the ISR
//            uses for its period arithmetic.
//
//    KNOWN   Remaining jitter sources, in order of size:
//            (1) Adafruit_NeoPixel's leds.show() bit-bangs inside
//                noInterrupts() — ~240µs with the clock ISR blocked, called
//                on every LED colour change. WS2812Serial (DMA) removes it.
//            (2) OLED redraws block loop() (~23ms at 400kHz I2C), which
//                jitters scheduled — not on-grid — pulses. Wire.setClock() at
//                1MHz would more than halve it.
//
//  v0.4  — 2026-07-03
//    ADDED   Per-port time shift: ±50ms in 0.5ms steps (5ms steps with the
//            encoder button held). Shift is always computed as an offset
//            from the reference grid — never by adjusting a port's tick
//            counter — so setting it back to 0 returns the port EXACTLY
//            to the grid, guaranteed. Long-press while editing Shift snaps
//            it to 0. Ports with a nonzero shift show a "*" marker on the
//            overview screen. Shifts survive resync (they are configuration,
//            not drift). Applied shift wraps modulo the port's pulse period
//            (shifting a periodic signal by a full period is identity), which
//            also guarantees at most one pending pulse per port.
//            Scheduled (shifted or swung) pulses fire from loop(), so their
//            jitter equals loop latency: microseconds typically, up to ~6ms
//            during an OLED redraw. A future hardware-timer scheduler can
//            remove that; on-grid (shift 0, no swing) pulses keep ISR timing.
//
//    RENAMED cvSwingTriggerTime[] → pulseTriggerTime[] and
//            updateCVSwingTriggers() → updateScheduledPulses(): the delayed-
//            pulse mechanism now serves time shift on both port types, not
//            just CV swing.
//
//    FIXED   Holding the nav encoder button as a coarse-adjust modifier no
//            longer triggers a menu navigation when released. (Pre-existing
//            bug: coarse tempo adjust always navigated the menu as a side
//            effect.)
//
//    UI      Parameter list is now a 3-row window that scrolls with the
//            selection (5 parameters no longer fit on the 64px display).
//
//    UI      Swing now shows as LOCKED on MIDI ports (matching the Division
//            lock) and cannot be edited there. Swing has only ever affected
//            CV pulses; the editable arrows on MIDI ports were misleading.
//            Swinging a MIDI clock stream requires unevenly-spaced ticks
//            (a per-port event queue) — a candidate for a future clock
//            engine, not supported by the current one-pending-pulse design.
//
//  v0.3  — 2026-07-03
//    FIXED   Nav encoder moved from pins 18,19 to 16,17. Pins 18/19 are
//            SDA/SCL for the Wire I2C bus on Teensy 4.1, which the OLED
//            uses — the encoder would have corrupted display communication.
//            (Fix ported from Clocktopus_v3.ino, along with the full-build
//            pin expansion notes in the PIN DEFINITIONS section.)
//
//    FIXED   micros() wraparound bug in updateCVPulseOff() and
//            updateCVSwingTriggers(). Plain `now >= target` comparisons fail
//            when micros() wraps (~every 71.6 min), which could leave a CV
//            pin stuck HIGH for over an hour mid-performance. Replaced with
//            the signed-difference idiom `(int32_t)(now - target) >= 0`.
//
//    FIXED   Blocking UART write in clockISR could deadlock: write() waits
//            for the UART interrupt when the TX buffer is full, but at ISR
//            priority 0 that interrupt can never run. Now guarded with
//            availableForWrite() — drops a byte instead of freezing.
//
//    FIXED   Red button LED moved from pin 26 to pin 28. Pin 26 has no PWM
//            on Teensy 4.1; analogWrite(26, 40) would write digital LOW and
//            the red "stopped" glow would never appear.
//
//    FIXED   Race in updateCVSwingTriggers(): between reading a trigger time
//            and clearing it, the ISR could schedule a new trigger which the
//            clear would erase (dropped pulse). Now compare-and-clear inside
//            a brief noInterrupts() section.
//
//    FIXED   resyncClock() now sends MIDI CONTINUE before re-arming the
//            timer, so the main-loop UART writes cannot race the ISR's.
//
//    FIXED   MIDI ports locked to the MIDI(24) division. Port 1's factory
//            default was "Quarter", which sends 1 clock/beat instead of 24 —
//            a receiver at 120 BPM would derive 5 BPM. Default corrected to
//            MIDI(24); switching any port to MIDI now forces MIDI(24); the
//            division parameter is read-only while a port's type is MIDI.
//            (Deliberate slow-clock tricks are sacrificed for a device that
//            syncs correctly out of the box.)
//
//    ADDED   USB MIDI clock output. The device now sends standard 24 PPQN
//            MIDI clock plus Start/Stop/Continue over USB, so a DAW can
//            sync to Clocktopus as an external clock source. The ISR only
//            counts pending ticks (usbMIDI.sendRealTime() can busy-wait on
//            a frozen millis() inside a priority-0 ISR — deadlock risk);
//            loop() drains the count to USB. USB clock jitter therefore
//            equals loop latency (µs typically, ~6ms worst case during an
//            OLED redraw) — fine for DAWs, which smooth incoming clock.
//            Divisions and swing do not apply to USB: DAWs expect the raw
//            24 PPQN stream and subdivide internally.
//
//  v0.2  — 2026-04-04
//    FIXED   Division table tick values were 4× too small. The internal clock
//            runs at 96 PPQN (ticks per quarter note), so a whole note needs
//            384 ticks (96×4), not 96. Every musical label in the old table
//            was shifted by two positions — a device set to "Quarter" was
//            actually outputting 16th notes. All values corrected.
//            (divisionTicks type upgraded from uint8_t to uint16_t to hold 384.)
//
//    FIXED   portTickCount[] type upgraded from uint8_t to uint16_t to match
//            the corrected division table (384 overflows a uint8_t).
//
//    FIXED   Swing delay base calculation was wrong. The code calculated
//            `pulseIntervalMicros × 12` and labelled it "one 8th note", but
//            12 ticks at 96 PPQN is a 32nd note. Swing delay was 4× too small
//            and nearly inaudible. Fixed to `pulseIntervalMicros × 48`
//            (= one 8th note = half a quarter note at 96 PPQN).
//
//    FIXED   Volca sync rate corrected to 2 PPQN (confirmed for Korg Volca
//            hardware sync port). At 96 PPQN internal, 2 PPQN = 48 ticks per
//            pulse (same period as an 8th note). Old code used 6 ticks and
//            labelled it "16th note rate", which was wrong for Volca.
//
//    FIXED   Race conditions: arrays shared between clockISR and loop() now
//            declared volatile so the compiler always reads/writes memory
//            directly rather than caching values in registers.
//            Affected: ledFlashTime[], cvPulseOffTime[], cvSwingTriggerTime[],
//            swingDelayMicros[], portTickCount[].
//
//    FIXED   clockTimer.priority(0) now set explicitly after clockTimer.begin()
//            in startClock() and resyncClock(). Priority 0 = highest on
//            Teensy 4.1, ensuring the clock ISR cannot be delayed by any
//            other interrupt. Timing accuracy is the core purpose of this device.
//
//    FIXED   Port type change (MIDI ↔ CV) now safely pauses the clock if it
//            is running, applies the change, then restarts. This prevents the
//            ISR from writing to a UART that is being re-initialised, which
//            could cause a crash. In practice users set port types before a
//            session and press stop before changing hardware connections anyway.
//
//    REMOVED Dead variable midiTickCount[8]. It was declared and reset in
//            start/resync but never read or written during clock operation.
//            Removing it cleans up 8 bytes of RAM and eliminates confusion.
//
//    NOTED   millis() inside clockISR (for ledFlashTime) is noted as safe on
//            Teensy 4.1 at high priority but may return a slightly stale value
//            (~1ms) if SysTick priority is lower. For LED flashing this is
//            imperceptible and accepted for now.
//
//  v0.1  — 2026-03-01
//    Initial build. 8-port MIDI/CV clock, per-port swing, NeoPixel LEDs,
//    OLED display, start/stop button with bicolour breathing LED, tap tempo.
// ============================================================================


// ============================================================================
//  INCLUDES
// ============================================================================

#include <IntervalTimer.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_NeoPixel.h>


// ============================================================================
//  PIN DEFINITIONS
// ============================================================================

// Eurorack CV output pins (connect via 74AHCT125 level shifter for 5V output)
const uint8_t CV_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};

// Button and switch pins
const uint8_t BTN_START_STOP = 20;   // Start/Stop/Resync button
const uint8_t SW_HALF        = 21;   // Speed toggle: half speed
const uint8_t SW_DOUBLE      = 22;   // Speed toggle: double speed
const uint8_t NAV_BTN_PIN    = 23;   // Nav encoder push button

// NeoPixel LED strip (all 8 LEDs on one pin)
const uint8_t LED_PIN   = 24;
const uint8_t LED_COUNT = 8;

// Start/Stop button bicolour LED pins
// Uses a bicolour (common-cathode) LED inserted into a frosted clear button cap.
// Green = clock running (breathing to BPM), Red = clock stopped (steady dim).
// Both pins must support analogWrite() / PWM on Teensy 4.1.
// NOTE: pin 26 is NOT PWM-capable on Teensy 4.1 — analogWrite() on it degrades
// to digital (values < 128 write LOW), so the dim red idle glow (40) would
// never light. Pin 28 is PWM-capable. Verify against the PJRC pinout card.
const uint8_t BTN_LED_GREEN = 25;   // PWM pin — green LED anode
const uint8_t BTN_LED_RED   = 28;   // PWM pin — red LED anode (was 26 — not PWM)

// Encoder pins (must be interrupt-capable pins on Teensy 4.1)
// Encoder 1: Tempo  — pins 30, 31
// Encoder 2: Nav    — pins 32, 33
// Pins 30–33 are claimed by no UART, so these survive a full 8-port build.
// Do NOT use 14/15/16/17 (Serial3/Serial4), and not 18/19 (I2C for the OLED).
//
// ⚠ FULL-BUILD PIN EXPANSION NOTE:
//   When all 8 MIDI UARTs are active, additional pin conflicts arise.
//   Teensy 4.1 UART TX/RX pairs (hardware-fixed):
//     Serial1 RX=0  TX=1  |  Serial2 RX=7  TX=8  |  Serial3 RX=15 TX=14
//     Serial4 RX=16 TX=17 |  Serial5 RX=21 TX=20  |  Serial6 RX=25 TX=24
//     Serial7 RX=28 TX=29 |  Serial8 RX=34 TX=35
//   Impact on this sketch when all 8 ports are active:
//     - Encoders: resolved in v0.6 — moved to 30,31 and 32,33, which no UART
//       claims. Nothing further to do for them.
//     - CV_PINS[6]=8 conflicts with Serial2 TX → change CV array (skip 7,8)
//     - BTN_START_STOP=20 conflicts with Serial5 TX → remap (e.g. 22)
//     - LED_PIN=24 conflicts with Serial6 TX → remap (NOT 33 — taken by the
//       nav encoder since v0.6; needs allocating against the final pinout)
//     - BTN_LED_GREEN=25 conflicts with Serial6 RX → remap (NOT 33, see above;
//       must be PWM-capable — 26 is NOT)
//     - BTN_LED_RED=28 conflicts with Serial7 RX → remap (e.g. 36 — must be PWM)
//   ⚠ These conflicts are NOT conditional on wiring anything up. initMIDIPorts()
//   calls begin() on all 8 UARTs unconditionally, and begin() reassigns the pin
//   mux for both RX and TX whether or not a cable is attached. An earlier version
//   of this note claimed the conflicts only mattered once Serial3–8 were wired;
//   that was wrong, and it hid the dead-encoder bug fixed in v0.6.


// ============================================================================
//  CONSTANTS
// ============================================================================

// MIDI message bytes
const uint8_t MIDI_CLOCK    = 0xF8;
const uint8_t MIDI_START    = 0xFA;
const uint8_t MIDI_STOP     = 0xFC;
const uint8_t MIDI_CONTINUE = 0xFB;

// CV pulse width in microseconds (5ms is standard for Eurorack)
const uint16_t CV_PULSE_WIDTH_US = 5000;

// LED flash duration in milliseconds
const uint16_t LED_FLASH_DURATION = 30;

// Start/Stop button LED brightness levels (0–255 for analogWrite)
// Green channel: breathes between these limits in sync with BPM when running.
// Red channel: steady dim glow when stopped, off when running.
const uint8_t BTN_LED_GREEN_MIN  =  8;   // dimmest point of the breathing cycle
const uint8_t BTN_LED_GREEN_MAX  = 220;  // brightest point of the breathing cycle
const uint8_t BTN_LED_RED_IDLE   = 40;   // steady dim red when clock is stopped

// Double-press window for resync (milliseconds)
const uint16_t DOUBLE_PRESS_MS = 400;

// Contact bounce guard (milliseconds). A mechanical switch produces several
// edges over 1–20ms; without this the extra edges land inside DOUBLE_PRESS_MS
// and a single press registers as a double press, triggering a resync.
const uint16_t BTN_DEBOUNCE_MS = 25;

// Long press threshold for nav encoder (milliseconds)
const uint16_t LONG_PRESS_MS = 600;

// Internal resolution: 96 PPQN (ticks per quarter note).
// 96 divides cleanly into all standard musical subdivisions:
//   Whole note  = 96 × 4 = 384 ticks
//   Half note   = 96 × 2 = 192 ticks
//   Quarter     = 96 × 1 =  96 ticks
//   8th note    = 96 / 2 =  48 ticks
//   16th note   = 96 / 4 =  24 ticks
//   32nd note   = 96 / 8 =  12 ticks
//   MIDI 24PPQ  = 96 / 24=   4 ticks
const uint8_t INTERNAL_PPQN = 96;


// ============================================================================
//  ENUMERATIONS
// ============================================================================

// UI navigation levels
enum UILevel {
  LEVEL_PORT_SELECT,   // Encoder 2 scrolls through ports 1-8
  LEVEL_PARAM_SELECT,  // Encoder 2 scrolls through parameters
  LEVEL_VALUE_EDIT     // Encoder 2 changes the selected value
};

// Per-port parameters available for editing
enum PortParam {
  PARAM_TYPE,      // MIDI or CV
  PARAM_DIVISION,  // Musical clock division
  PARAM_SHIFT,     // Time shift in ms (offset from the reference grid)
  PARAM_SWING,     // Swing percentage
  PARAM_ENABLED,   // Port on/off
  PARAM_COUNT      // Always last — used for wrap-around arithmetic
};

// Port output type
enum PortType {
  PORT_MIDI,
  PORT_CV
};


// ============================================================================
//  DATA STRUCTURES
// ============================================================================

// Configuration for one port
struct PortConfig {
  PortType type;      // MIDI or CV
  uint8_t  division;  // Index into divisionNames[] / divisionTicks[]
  uint8_t  swing;     // Swing percentage: 50 = straight, 90 = heavy swing
  int8_t   shift;     // Time shift in 0.5ms steps: -100…+100 = ±50ms, 0 = on grid
  bool     enabled;   // Whether this port is active
};


// ============================================================================
//  CLOCK DIVISION TABLES
// ============================================================================
//
//  divisionTicks[] = how many internal ticks (at 96 PPQN) before firing a pulse.
//  The internal timer fires 96 times per quarter note, so:
//
//   384 ticks = whole note   (4 beats × 96)
//   192 ticks = half note    (2 beats × 96)
//    96 ticks = quarter note (1 beat  × 96)
//    48 ticks = 8th note     (half beat)
//    24 ticks = 16th note    (quarter beat)
//    12 ticks = 32nd note    (eighth beat)
//     4 ticks = MIDI 24PPQN  (96 ÷ 24 — raw standard MIDI clock rate)
//    48 ticks = Volca sync   (2 PPQN = 2 pulses per quarter note = 96 ÷ 2)
//               Note: Volca hardware sync port expects 2 PPQN, confirmed for
//               Korg Volca series. Same tick count as an 8th note but labelled
//               separately for clarity in the UI.
//
//  IMPORTANT: divisionTicks[] uses uint16_t because 384 overflows uint8_t (max 255).
//  portTickCount[] must also be uint16_t for the same reason.

const uint8_t NUM_DIVISIONS = 8;

const char* divisionNames[NUM_DIVISIONS] = {
  "Whole",     // index 0 — 1 pulse per bar (4/4)
  "Half",      // index 1 — 2 pulses per bar
  "Quarter",   // index 2 — 4 pulses per bar (1 per beat)
  "8th",       // index 3 — 8 pulses per bar
  "16th",      // index 4 — 16 pulses per bar
  "32nd",      // index 5 — 32 pulses per bar
  "MIDI(24)",  // index 6 — raw 24 PPQN MIDI clock
  "Volca"      // index 7 — Korg Volca hardware sync (2 PPQN)
};

// FIXED v0.2: All values corrected. Was 4× too small due to misreading PPQN.
// Type is uint16_t — 384 does not fit in uint8_t.
const uint16_t divisionTicks[NUM_DIVISIONS] = {
  384,  // whole note   (96 × 4)
  192,  // half note    (96 × 2)
   96,  // quarter note (96 × 1)
   48,  // 8th note     (96 / 2)
   24,  // 16th note    (96 / 4)
   12,  // 32nd note    (96 / 8)
    4,  // MIDI 24PPQN  (96 / 24)
   48   // Volca 2PPQN  (96 / 2) — same period as 8th note
};

// The only valid division for a MIDI-type port. MIDI receivers derive tempo
// by assuming 24 clocks per quarter note; any other rate makes the follower
// run at the wrong tempo (e.g. "Quarter" = 1/24th of the dialled BPM).
const uint8_t DIV_MIDI24 = 6;  // index into divisionTicks[] / divisionNames[]

const char* paramNames[PARAM_COUNT] = {"Type", "Division", "Shift", "Swing", "Enabled"};


// ============================================================================
//  GLOBAL STATE — CLOCK
// ============================================================================

volatile float    bpm                 = 120.0f;
// Internal tick period, kept in two forms — both recalculated by updateBPM().
//
//   pulseIntervalMicros   integer µs. Used ONLY for the ISR's period arithmetic
//                         (shift/swing wrapping), where sub-µs precision is
//                         irrelevant and integer maths keeps the ISR cheap.
//
//   pulseIntervalMicrosF  exact µs, handed to IntervalTimer. The PIT is clocked
//                         at 24MHz, so it resolves 1/24 µs — and
//                         IntervalTimer::cyclesFromPeriod() has separate code
//                         paths: an integer period truncates (24*P - 1), a float
//                         period rounds to nearest (24.0f*P - 0.5f). At 120 BPM
//                         the exact period is 5208.333µs; passing 5208 loaded
//                         124992 cycles instead of 125000 — 0.0064% fast, about
//                         19ms of accumulated drift per 5 minutes against a DAW.
//                         Passing the float lands on the exact reload value.
volatile uint32_t pulseIntervalMicros  = 5208;
float             pulseIntervalMicrosF = 5208.3333f;
volatile bool     clockRunning        = false;
volatile uint32_t internalTickCount   = 0;       // 0–95, wraps at INTERNAL_PPQN

float speedMultiplier = 1.0f;  // 0.5 / 1.0 / 2.0 depending on toggle switch

// USB MIDI clock handoff: the ISR produces ticks, loop() consumes them.
// Two separate counters (each with a single writer) instead of one shared
// counter, because `pending--` in loop() would be a read-modify-write that
// an ISR increment could interleave with and get lost. Equality comparison
// of two independently-written uint32s is race-free.
volatile uint32_t usbClockProduced = 0;  // written only by clockISR
uint32_t          usbClockConsumed = 0;  // written only by loop()


// ============================================================================
//  GLOBAL STATE — PORTS
// ============================================================================

// Default configuration for first power-on.
// Division indices refer to divisionTicks[] above:
//   0=Whole, 1=Half, 2=Quarter, 3=8th, 4=16th, 5=32nd, 6=MIDI(24), 7=Volca
PortConfig ports[8] = {
  {PORT_MIDI, 6, 50, 0, true},   // port 1: MIDI out, MIDI(24) — MIDI ports are locked to this rate
  {PORT_CV,   2, 50, 0, true},   // port 2: CV,  quarter note
  {PORT_CV,   3, 50, 0, true},   // port 3: CV,  8th note
  {PORT_CV,   4, 50, 0, true},   // port 4: CV,  16th note
  {PORT_CV,   1, 50, 0, true},   // port 5: CV,  half note
  {PORT_CV,   0, 50, 0, true},   // port 6: CV,  whole note
  {PORT_CV,   2, 50, 0, true},   // port 7: CV,  quarter note
  {PORT_CV,   2, 50, 0, true},   // port 8: CV,  quarter note
};

// Per-port tick counters for clock division.
// FIXED v0.2: uint16_t — must hold up to 384 (whole note ticks).
// volatile — written by clockISR, read by editCurrentValue in loop().
volatile uint16_t portTickCount[8] = {0};

// Per-port CV pulse-off times (micros timestamp when pulse should end).
// volatile — written by clockISR and updateScheduledPulses, read by updateCVPulseOff.
volatile uint32_t cvPulseOffTime[8] = {0};

// Per-port swing: 8th-note phase tracker (0 = downbeat, 1 = upbeat).
// Written and read only in clockISR — no volatile needed, but kept for clarity.
uint8_t eighthNotePhase[8] = {0};

// Pre-computed swing delay per port in microseconds.
// volatile — written by updateSwingDelays in loop(), read by clockISR.
volatile uint32_t swingDelayMicros[8] = {0};

// Pre-computed effective time shift per port, in microseconds, always in
// [0, port period). Negative user shifts are folded to their positive
// equivalent (firing 10ms early = firing period−10ms after the previous
// grid tick), so the ISR only ever schedules forward in time.
// volatile — written by updateShiftDelays in loop(), read by clockISR.
volatile uint32_t shiftDelayMicros[8] = {0};

// Per-port scheduled pulse times (time-shifted pulses, and swing-delayed
// CV upbeats). One pending event per port — the ISR keeps total delay
// under one period, so a second event can never queue behind the first.
// volatile — written by clockISR, read and cleared by updateScheduledPulses in loop().
volatile uint32_t pulseTriggerTime[8] = {0};


// ============================================================================
//  GLOBAL STATE — UI
// ============================================================================

UILevel   currentLevel       = LEVEL_PORT_SELECT;
uint8_t   selectedPort       = 0;         // 0–7
PortParam selectedParam      = PARAM_TYPE;
bool      displayNeedsUpdate = true;


// ============================================================================
//  GLOBAL STATE — LEDS
// ============================================================================

// millis() timestamp of last pulse per port.
// volatile — written by clockISR and updateScheduledPulses, read by updateAllLEDs.
volatile uint32_t ledFlashTime[8] = {0};


// ============================================================================
//  GLOBAL STATE — START/STOP BUTTON LED
//
//  The button contains a bicolour LED (green/red) behind a frosted clear cap.
//  When running: green channel breathes in a sine curve locked to quarter-note BPM.
//  When stopped: red channel holds a steady dim glow.
//
//  How the sine breathing works:
//    phase = elapsed time within current beat / quarter-note duration  (0.0–1.0)
//    brightness = sin(phase × π)  maps 0→0, 0.5→1.0 (peak), 1.0→0
//    This produces one smooth arch per beat, peaking at the midpoint.
//    Scaled into BTN_LED_GREEN_MIN … BTN_LED_GREEN_MAX so the LED always
//    has a faint glow even at the trough of the cycle.
// ============================================================================

uint32_t btnLedCycleStart  = 0;

// Duration of one quarter-note in microseconds — updated whenever BPM changes.
// volatile — written by updateBPM (loop), read by updateButtonLED (loop).
// Both accesses are in loop() so volatile is not strictly required here,
// but it documents that pulseIntervalMicros feeds into it indirectly via the ISR.
volatile uint32_t quarterNoteMicros = 500000;  // default: 120 BPM


// ============================================================================
//  GLOBAL STATE — BUTTONS
// ============================================================================

// Start/Stop button
bool     btnLastState      = HIGH;
uint32_t btnLastEdgeTime   = 0;      // last accepted edge — debounce reference
uint32_t lastPressTime     = 0;
bool     btnPendingSingle  = false;  // single-press deferred until double-press window expires

// Nav encoder button
uint32_t navLastEdgeTime  = 0;       // last accepted edge — debounce reference
uint32_t navPressTime     = 0;
bool     navButtonHeld    = false;
// Set when an encoder is turned while the button is held (coarse-adjust
// modifier). The release is then swallowed instead of navigating the menu.
bool     navButtonUsedAsModifier = false;

// ============================================================================
//  HARDWARE OBJECTS
// ============================================================================

IntervalTimer clockTimer;

Encoder tempoEncoder(30, 31);  // Encoder 1: Tempo
Encoder navEncoder(32, 33);    // Encoder 2: Port config navigation

long lastTempoEncoderPos = 0;
long lastNavEncoderPos   = 0;

Adafruit_SH1106G display(128, 64, &Wire, -1);

Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// MIDI output ports — Teensy 4.1 hardware UARTs
HardwareSerial* midiPorts[8] = {
  &Serial1, &Serial2, &Serial3, &Serial4,
  &Serial5, &Serial6, &Serial7, &Serial8
};


// ============================================================================
//  CLOCK CALCULATIONS
// ============================================================================

// Call whenever BPM or speedMultiplier changes.
void updateBPM(float newBPM) {
  bpm = constrain(newBPM, 20.0f, 300.0f);
  // Internal tick interval:
  //   60,000,000 µs/min  ÷  (BPM × speedMultiplier × INTERNAL_PPQN)
  pulseIntervalMicrosF = 60000000.0f /
                         (bpm * speedMultiplier * (float)INTERNAL_PPQN);
  // Round rather than truncate — the integer copy is only used for the ISR's
  // period arithmetic, but there is no reason for it to be biased low.
  pulseIntervalMicros  = (uint32_t)(pulseIntervalMicrosF + 0.5f);
  // Quarter-note duration for the button LED breathing cycle:
  //   60,000,000 µs/min  ÷  (BPM × speedMultiplier)
  quarterNoteMicros = (uint32_t)(60000000.0f / (bpm * speedMultiplier));
  // Both delay tables are derived from pulseIntervalMicros — recompute here so
  // no caller can change tempo and leave them stale.
  updateSwingDelays();
  updateShiftDelays();
}

// Recompute swing delay for every port.
// Call after any BPM change or after editing a port's swing value.
void updateSwingDelays() {
  // Duration of one 8th note in microseconds at current tempo.
  // One 8th note = half a quarter note = INTERNAL_PPQN/2 ticks.
  // FIXED v0.2: was (INTERNAL_PPQN / 8) = 12 ticks, which is a 32nd note —
  // 4× too short, making swing nearly inaudible. Correct divisor is 2.
  uint32_t eighthNoteMicros = pulseIntervalMicros * (INTERNAL_PPQN / 2);  // × 48
  for (int i = 0; i < 8; i++) {
    // swing 50% = straight (0 delay)
    // swing 67% = moderate swing (upbeat delayed by ~33% of an 8th note)
    // swing 90% = very heavy swing
    float swingFrac = (ports[i].swing - 50) / 100.0f;  // 0.0 – 0.4
    swingDelayMicros[i] = (uint32_t)(eighthNoteMicros * swingFrac * 2.0f);
  }
}

// Recompute the effective time-shift delay for every port.
// Call after any BPM change, or after editing a port's shift or division
// (both change the shift's relationship to the port's pulse period).
//
// The user's shift is folded into [0, period): a negative shift (fire early)
// becomes an equivalent positive delay from the PREVIOUS grid tick, and a
// shift beyond one period wraps (shifting a periodic pulse train by a full
// period is indistinguishable from not shifting it). The result: the ISR
// only ever schedules forward, and at most one pulse is pending per port.
void updateShiftDelays() {
  for (int i = 0; i < 8; i++) {
    int32_t periodUs = (int32_t)(pulseIntervalMicros * divisionTicks[ports[i].division]);
    int32_t shiftUs  = (int32_t)ports[i].shift * 500;  // 0.5ms steps → µs
    int32_t eff = shiftUs % periodUs;
    if (eff < 0) eff += periodUs;
    shiftDelayMicros[i] = (uint32_t)eff;
  }
}


// ============================================================================
//  MIDI PORT HELPERS
// ============================================================================

void initMIDIPorts() {
  for (int i = 0; i < 8; i++) {
    midiPorts[i]->begin(31250);  // MIDI baud rate — must be exactly 31250
  }
}

// Send a byte to a single port (only if configured as MIDI and enabled).
inline void sendMIDI(uint8_t portIndex, uint8_t msg) {
  if (ports[portIndex].enabled && ports[portIndex].type == PORT_MIDI) {
    midiPorts[portIndex]->write(msg);
  }
}

// Send a byte to ALL enabled MIDI ports (used for START/STOP/CONTINUE).
void sendToAllMIDIPorts(uint8_t msg) {
  for (int i = 0; i < 8; i++) {
    sendMIDI(i, msg);
  }
}


// ============================================================================
//  USB MIDI OUTPUT
//
//  The USB connection acts as a ninth clock output, fixed at the raw
//  24 PPQN MIDI clock rate (no division/swing — DAWs subdivide internally).
//  Transport messages are sent directly from loop() context; clock ticks
//  are counted by the ISR and drained here. See the v0.3 changelog entry
//  for why the ISR must not call usbMIDI itself.
// ============================================================================

// Called from main loop — sends any USB clock ticks the ISR has counted.
void updateUSBMIDI() {
  bool sent = false;
  while (usbClockConsumed != usbClockProduced) {
    usbMIDI.sendRealTime(usbMIDI.Clock);
    usbClockConsumed++;
    sent = true;
  }
  if (sent) usbMIDI.send_now();  // flush immediately — don't wait for USB buffering

  // Drain and discard any incoming USB MIDI so the receive buffer never fills.
  while (usbMIDI.read()) {}
}


// ============================================================================
//  CV OUTPUT HELPERS
// ============================================================================

void initCVPins() {
  for (int i = 0; i < 8; i++) {
    pinMode(CV_PINS[i], OUTPUT);
    digitalWriteFast(CV_PINS[i], LOW);
  }
}

// Called from main loop — turns off any CV pulses whose time has elapsed.
void updateCVPulseOff() {
  uint32_t now = micros();
  for (int i = 0; i < 8; i++) {
    // Signed-difference comparison is wraparound-safe. micros() wraps every
    // ~71.6 minutes; a plain `now >= offTime` scheduled just before the wrap
    // would stay false for over an hour, leaving the CV pin stuck HIGH.
    if (cvPulseOffTime[i] > 0 && (int32_t)(now - cvPulseOffTime[i]) >= 0) {
      digitalWriteFast(CV_PINS[i], LOW);
      cvPulseOffTime[i] = 0;
    }
  }
}

// Called from main loop — fires any scheduled pulses whose time has come:
// time-shifted pulses (either port type) and swing-delayed CV upbeats.
//
// Timing note: because this runs from loop(), scheduled pulses jitter by
// the loop latency — microseconds normally, up to ~6ms while the OLED is
// redrawing. On-grid pulses (shift 0, no swing) fire in the ISR and are
// unaffected. A dedicated hardware-timer scheduler could remove this.
void updateScheduledPulses() {
  uint32_t now = micros();
  for (int i = 0; i < 8; i++) {
    uint32_t t = pulseTriggerTime[i];
    // Signed-difference comparison — wraparound-safe (see updateCVPulseOff).
    if (t > 0 && (int32_t)(now - t) >= 0) {
      if (ports[i].enabled) {
        if (ports[i].type == PORT_CV) {
          digitalWriteFast(CV_PINS[i], HIGH);
          uint32_t off      = now + CV_PULSE_WIDTH_US;   // 0 = "no pulse pending"
          cvPulseOffTime[i] = (off == 0) ? 1 : off;
        } else {
          // Same guard as the ISR path: never block on a full TX buffer.
          if (midiPorts[i]->availableForWrite() > 0) {
            midiPorts[i]->write(MIDI_CLOCK);
          }
        }
        ledFlashTime[i] = millis();
      }
      // Clear only if the ISR hasn't scheduled a NEW trigger since we read t —
      // blindly writing 0 could erase a fresh trigger and drop a pulse.
      // noInterrupts() (PRIMASK) blocks even the priority-0 clock ISR, making
      // the compare-and-clear atomic; the window is a few CPU cycles.
      noInterrupts();
      if (pulseTriggerTime[i] == t) pulseTriggerTime[i] = 0;
      interrupts();
    }
  }
}


// ============================================================================
//  CLOCK ISR  (runs at INTERNAL_PPQN rate — 96 times per quarter note)
//
//  KEEP THIS FAST. No Serial.print, no floating point, no display updates.
//  Record timestamps here; do slow work in loop().
//
//  Priority set to 0 (highest on Teensy 4.1) in startClock() / resyncClock().
//  This ensures the timer cannot be delayed by UART, I2C, NeoPixel, or any
//  other peripheral interrupt — timing accuracy is the whole point.
// ============================================================================

void FASTRUN clockISR() {
  if (!clockRunning) return;

  uint32_t now = micros();

  // USB MIDI clock: 24 PPQN = every 4th internal tick (96 / 24).
  // Only count here — loop() sends. Calling usbMIDI from this ISR could
  // busy-wait forever: its timeout needs millis() to advance, and SysTick
  // cannot preempt priority 0. (96 % 4 == 0, so the wrap keeps alignment.)
  if ((internalTickCount & 3u) == 0) {
    usbClockProduced++;
  }

  for (int i = 0; i < 8; i++) {
    if (!ports[i].enabled) continue;

    portTickCount[i]++;

    bool shouldFire = (portTickCount[i] >= divisionTicks[ports[i].division]);

    if (shouldFire) {
      portTickCount[i] = 0;

      // Swing: alternate downbeat / upbeat on each successive firing.
      // Swing delay is only applied to upbeats (eighthNotePhase == 1).
      bool isUpbeat = (eighthNotePhase[i] == 1);
      eighthNotePhase[i] ^= 1;

      // Total delay from the grid for this pulse: the port's time shift,
      // plus swing on CV upbeats. `now` IS the grid — shift is always an
      // offset from it, never an adjustment to the tick counter, so setting
      // shift back to 0 lands the port exactly back on the grid.
      uint32_t fireDelay = shiftDelayMicros[i];
      if (ports[i].type == PORT_CV && isUpbeat) {
        fireDelay += swingDelayMicros[i];
      }
      if (fireDelay > 0) {
        // Wrap into one period so at most one pulse is ever pending per
        // port (pulseTriggerTime[] holds a single event per port).
        uint32_t period = pulseIntervalMicros * divisionTicks[ports[i].division];
        fireDelay %= period;
      }

      if (fireDelay > 0) {
        // Schedule — updateScheduledPulses() in loop() fires it.
        // 0 is the "nothing pending" sentinel, and `now + fireDelay` really
        // does land on 0 once per micros() wrap (~71.6 min), which would
        // silently drop the pulse. Nudging to 1µs is imperceptible.
        uint32_t t = now + fireDelay;
        pulseTriggerTime[i] = (t == 0) ? 1 : t;

      } else if (ports[i].type == PORT_MIDI) {
        // On-grid: send MIDI clock byte immediately via hardware UART.
        // Guard: write() blocks when the TX buffer is full, and at ISR
        // priority 0 the UART interrupt can never preempt us to drain it —
        // that would deadlock the device. Dropping one clock byte under
        // pathological load is the lesser evil.
        if (midiPorts[i]->availableForWrite() > 0) {
          midiPorts[i]->write(MIDI_CLOCK);
        }
        // millis() is safe here on Teensy 4.1 but may be ~1ms stale at
        // high ISR priority. Imperceptible for LED flash purposes.
        ledFlashTime[i] = millis();

      } else {
        // On-grid: fire the CV pulse immediately.
        digitalWriteFast(CV_PINS[i], HIGH);
        // 0 means "no pulse pending" — see the sentinel note above. Here the
        // cost of losing the timestamp is a CV pin latched HIGH until the
        // port's next pulse.
        uint32_t off      = now + CV_PULSE_WIDTH_US;
        cvPulseOffTime[i] = (off == 0) ? 1 : off;
        ledFlashTime[i]   = millis();
      }
    }
  }

  // Advance global tick counter (used for future features; not driving outputs directly)
  internalTickCount = (internalTickCount + 1) % INTERNAL_PPQN;
}


// ============================================================================
//  CLOCK TRANSPORT
// ============================================================================

void startClock() {
  if (clockRunning) return;

  // Reset all per-port counters for a clean start from beat 1.
  internalTickCount = 0;
  for (int i = 0; i < 8; i++) {
    // Pre-load so every port's first pulse lands on the very first ISR tick,
    // in phase with the USB 24 PPQN stream (which fires when
    // internalTickCount == 0). Starting from 0 delayed each port by its whole
    // division before the first pulse — a MIDI port trailed USB by 3 internal
    // ticks (15.6ms at 120 BPM), and that skew persisted for the whole session.
    portTickCount[i]      = divisionTicks[ports[i].division] - 1;
    eighthNotePhase[i]    = 0;
    pulseTriggerTime[i]   = 0;
    cvPulseOffTime[i]     = 0;
  }

  // Reset the USB clock handoff counters (safe: ISR is not running yet).
  usbClockProduced = 0;
  usbClockConsumed = 0;

  clockRunning = true;
  sendToAllMIDIPorts(MIDI_START);
  usbMIDI.sendRealTime(usbMIDI.Start);
  usbMIDI.send_now();
  clockTimer.begin(clockISR, pulseIntervalMicrosF);
  // FIXED v0.2: Explicitly set highest interrupt priority.
  // Without this, Teensyduino's default IntervalTimer priority may allow
  // UART or other ISRs to delay the clock, introducing jitter.
  clockTimer.priority(0);

  displayNeedsUpdate = true;
}

void stopClock() {
  if (!clockRunning) return;

  clockRunning = false;
  clockTimer.end();
  sendToAllMIDIPorts(MIDI_STOP);
  usbMIDI.sendRealTime(usbMIDI.Stop);
  usbMIDI.send_now();

  // Kill all CV outputs immediately.
  for (int i = 0; i < 8; i++) {
    digitalWriteFast(CV_PINS[i], LOW);
    cvPulseOffTime[i]     = 0;
    pulseTriggerTime[i]   = 0;
  }

  displayNeedsUpdate = true;
}

void resyncClock() {
  // Restart the clock from beat 1 without sending STOP/START.
  // Used for tight re-alignment mid-session (double-press of start/stop button).
  clockTimer.end();

  internalTickCount = 0;
  for (int i = 0; i < 8; i++) {
    // See startClock() — pre-load keeps DIN, CV and USB outputs in phase.
    portTickCount[i]      = divisionTicks[ports[i].division] - 1;
    eighthNotePhase[i]    = 0;
    pulseTriggerTime[i]   = 0;
  }

  // Send CONTINUE before re-arming the timer: once the ISR is live it also
  // writes to these UARTs, and HardwareSerial isn't safe against concurrent
  // writes from loop() and a higher-priority ISR. (MIDI also specifies
  // Continue arrives before the clock resumes.)
  sendToAllMIDIPorts(MIDI_CONTINUE);
  usbMIDI.sendRealTime(usbMIDI.Continue);
  usbMIDI.send_now();

  // Reset the USB clock handoff counters (safe: timer is stopped here).
  usbClockProduced = 0;
  usbClockConsumed = 0;

  clockTimer.begin(clockISR, pulseIntervalMicrosF);
  // FIXED v0.2: Re-apply priority after re-arming the timer.
  clockTimer.priority(0);

  displayNeedsUpdate = true;
}


// ============================================================================
//  ENCODER 1 — TEMPO
// ============================================================================

void handleTempoEncoder() {
  long pos   = tempoEncoder.read();
  long delta = pos - lastTempoEncoderPos;

  if (abs(delta) >= 4) {  // 4 counts per detent on most encoders
    int steps = delta / 4;
    lastTempoEncoderPos = pos;

    // Hold nav encoder button while turning tempo encoder for coarse adjust:
    // 5 BPM per step instead of 0.5 BPM.
    bool coarse = (digitalRead(NAV_BTN_PIN) == LOW);
    if (coarse) navButtonUsedAsModifier = true;  // don't navigate on release
    float increment = coarse ? 5.0f : 0.5f;
    updateBPM(bpm + (steps * increment));

    if (clockRunning) {
      clockTimer.update(pulseIntervalMicrosF);  // adjust rate without restarting
    }

    displayNeedsUpdate = true;
  }
}


// ============================================================================
//  ENCODER 2 — PORT CONFIG NAVIGATION
// ============================================================================

// Apply one or more encoder steps to the currently selected parameter value.
void editCurrentValue(int steps) {
  PortConfig& p = ports[selectedPort];

  switch (selectedParam) {

    case PARAM_TYPE: {
      // Toggle between MIDI and CV.
      // FIXED v0.2: If the clock is running, pause it before changing port type.
      // Switching a UART mid-transmission (or reconfiguring a CV pin while the
      // ISR might be writing to it) can cause a crash. In practice users set
      // port types during setup, not mid-performance — a brief pause is fine.
      bool wasRunning = clockRunning;
      if (wasRunning) stopClock();

      p.type = (p.type == PORT_MIDI) ? PORT_CV : PORT_MIDI;

      if (p.type == PORT_MIDI) {
        // Switching TO MIDI: reinitialise UART, stop driving the CV pin.
        midiPorts[selectedPort]->begin(31250);
        digitalWriteFast(CV_PINS[selectedPort], LOW);
        pinMode(CV_PINS[selectedPort], INPUT);  // release the pin
        // MIDI ports are locked to the standard 24 PPQN rate — any carried-over
        // CV division would make the receiving device run at the wrong tempo.
        p.division = DIV_MIDI24;
      } else {
        // Switching TO CV: set pin as output, ensure it starts LOW.
        pinMode(CV_PINS[selectedPort], OUTPUT);
        digitalWriteFast(CV_PINS[selectedPort], LOW);
      }

      // Reset this port's tick counter so the new type takes effect cleanly.
      portTickCount[selectedPort] = 0;
      updateShiftDelays();  // division may have changed with the type

      if (wasRunning) startClock();
      break;
    }

    case PARAM_DIVISION:
      // Division is locked on MIDI ports (always MIDI(24) — see DIV_MIDI24).
      if (p.type == PORT_MIDI) break;
      p.division = (uint8_t)constrain((int)p.division + steps, 0, NUM_DIVISIONS - 1);
      // Reset tick counter so new division takes effect at next beat boundary.
      portTickCount[selectedPort] = 0;
      updateShiftDelays();  // period changed — refold the shift into it
      break;

    case PARAM_SHIFT: {
      // 0.5ms per detent; hold the encoder button while turning for 5ms steps.
      int stepSize = (digitalRead(NAV_BTN_PIN) == LOW) ? 10 : 1;
      p.shift = (int8_t)constrain((int)p.shift + steps * stepSize, -100, 100);
      updateShiftDelays();
      break;
    }

    case PARAM_SWING:
      // Swing is locked on MIDI ports — it only affects CV pulses. Swinging
      // a MIDI clock stream needs unevenly-spaced ticks (many pulses in
      // flight per port with different delays), which the one-pending-event
      // scheduler cannot represent. See the swing notes in clockISR.
      if (p.type == PORT_MIDI) break;
      p.swing = (uint8_t)constrain((int)p.swing + steps, 50, 90);
      updateSwingDelays();
      break;

    case PARAM_ENABLED:
      p.enabled = !p.enabled;
      if (!p.enabled && p.type == PORT_CV) {
        digitalWriteFast(CV_PINS[selectedPort], LOW);
      }
      break;

    default:
      break;
  }
}

void handleNavEncoder() {
  long pos   = navEncoder.read();
  long delta = pos - lastNavEncoderPos;

  if (abs(delta) >= 4) {
    int steps = delta / 4;
    lastNavEncoderPos = pos;

    // Turning while the button is held = coarse-adjust modifier in use;
    // the release must not be treated as a navigation press.
    if (digitalRead(NAV_BTN_PIN) == LOW) navButtonUsedAsModifier = true;

    switch (currentLevel) {
      case LEVEL_PORT_SELECT:
        selectedPort = (uint8_t)((selectedPort + steps + 8) % 8);
        break;

      case LEVEL_PARAM_SELECT:
        selectedParam = (PortParam)((selectedParam + steps + PARAM_COUNT) % PARAM_COUNT);
        break;

      case LEVEL_VALUE_EDIT:
        editCurrentValue(steps);
        break;
    }

    displayNeedsUpdate = true;
  }
}

void handleNavButton() {
  bool pressed = (digitalRead(NAV_BTN_PIN) == LOW);
  uint32_t now = millis();

  // navButtonHeld is the debounced state. Ignore no-change polls, and ignore
  // any edge arriving while the contacts are still bouncing — otherwise a
  // single press can register as press-release-press and skip a menu level.
  if (pressed == navButtonHeld) return;
  if ((now - navLastEdgeTime) < BTN_DEBOUNCE_MS) return;
  navLastEdgeTime = now;

  // Button just pressed — record time.
  if (pressed) {
    navPressTime  = now;
    navButtonHeld = true;

  } else {
    // Button just released — determine short vs long press.
    uint32_t heldFor = now - navPressTime;
    navButtonHeld    = false;

    if (navButtonUsedAsModifier) {
      // Button was held as a coarse-adjust modifier while an encoder turned —
      // that's not a navigation press. Swallow it.
      navButtonUsedAsModifier = false;

    } else if (heldFor >= LONG_PRESS_MS) {
      if (currentLevel == LEVEL_VALUE_EDIT && selectedParam == PARAM_SHIFT) {
        // Long press while editing Shift: snap the port back onto the grid.
        ports[selectedPort].shift = 0;
        updateShiftDelays();
      } else {
        // Long press — escape back to top level from anywhere.
        currentLevel = LEVEL_PORT_SELECT;
      }
    } else {
      // Short press — advance deeper into the menu hierarchy.
      switch (currentLevel) {
        case LEVEL_PORT_SELECT:
          currentLevel  = LEVEL_PARAM_SELECT;
          selectedParam = PARAM_TYPE;
          break;

        case LEVEL_PARAM_SELECT:
          currentLevel = LEVEL_VALUE_EDIT;
          break;

        case LEVEL_VALUE_EDIT:
          // Confirm current value and return to parameter list.
          currentLevel = LEVEL_PARAM_SELECT;
          break;
      }
    }

    displayNeedsUpdate = true;
  }
}


// ============================================================================
//  START / STOP BUTTON
// ============================================================================

void handleStartStopButton() {
  bool state   = digitalRead(BTN_START_STOP);
  uint32_t now = millis();

  // Only act on a state change that has settled — bounce edges are ignored,
  // and btnLastState latches only on an accepted edge so the guard holds.
  if (state != btnLastState && (now - btnLastEdgeTime) >= BTN_DEBOUNCE_MS) {
    btnLastEdgeTime = now;
    btnLastState    = state;

    if (state == LOW) {  // falling edge = press
      if (btnPendingSingle && (now - lastPressTime < DOUBLE_PRESS_MS)) {
        // Second press within window — cancel the pending single-press and resync.
        btnPendingSingle = false;
        if (clockRunning) resyncClock();
      } else {
        // First press — defer action until the double-press window expires.
        btnPendingSingle = true;
      }
      lastPressTime = now;
    }
  }

  // Single-press window expired: now safe to act on it as a plain toggle.
  if (btnPendingSingle && (now - lastPressTime >= DOUBLE_PRESS_MS)) {
    btnPendingSingle = false;
    if (clockRunning) stopClock();
    else              startClock();
  }
}


// ============================================================================
//  START/STOP BUTTON LED — BREATHING GLOW
//
//  Call from loop() on every iteration. Non-blocking.
//
//  When RUNNING:
//    Green channel breathes in a sine curve over one quarter-note period.
//    The sine wave gives an organic feel — slow fade in, peak at the beat
//    midpoint, slow fade out — rather than a mechanical linear ramp.
//    Red channel is fully off.
//
//  When STOPPED:
//    Red channel holds a steady dim glow (BTN_LED_RED_IDLE).
//    Green channel is off.
// ============================================================================

void updateButtonLED() {
  if (clockRunning) {
    uint32_t now     = micros();
    uint32_t elapsed = now - btnLedCycleStart;

    // Roll over to next beat cycle if we've passed the end of this one.
    // Advancing by exactly quarterNoteMicros (rather than resetting to now)
    // keeps the breathing locked to tempo even if loop() occasionally runs slow.
    if (elapsed >= quarterNoteMicros) {
      btnLedCycleStart += quarterNoteMicros;
      elapsed = now - btnLedCycleStart;
      // Guard against multiple missed beats (e.g. after a long OLED redraw).
      if (elapsed >= quarterNoteMicros) {
        btnLedCycleStart = now;
        elapsed = 0;
      }
    }

    // Phase 0.0 (beat start) → 1.0 (beat end)
    float btnLedPhase = (float)elapsed / (float)quarterNoteMicros;

    // sin(0) = 0, sin(π/2) = 1, sin(π) = 0 → one smooth arch per beat
    float sinVal = sinf(btnLedPhase * 3.14159265f);

    uint8_t brightness = (uint8_t)(
      BTN_LED_GREEN_MIN + sinVal * (BTN_LED_GREEN_MAX - BTN_LED_GREEN_MIN)
    );

    analogWrite(BTN_LED_GREEN, brightness);
    analogWrite(BTN_LED_RED,   0);

  } else {
    // Stopped: steady dim red, green off.
    analogWrite(BTN_LED_GREEN, 0);
    analogWrite(BTN_LED_RED,   BTN_LED_RED_IDLE);

    // Reset so breathing starts cleanly from the beginning on next startClock().
    btnLedCycleStart = micros();
  }
}


// ============================================================================
//  SPEED MULTIPLIER SWITCH
//  3-position toggle: HALF — NORMAL — DOUBLE
//  Wire centre pin to GND; two outer pins to SW_HALF and SW_DOUBLE with INPUT_PULLUP.
// ============================================================================

void handleSpeedSwitch() {
  float newMult = 1.0f;

  if      (digitalRead(SW_HALF)   == LOW) newMult = 0.5f;
  else if (digitalRead(SW_DOUBLE) == LOW) newMult = 2.0f;

  if (newMult != speedMultiplier) {
    speedMultiplier = newMult;
    updateBPM(bpm);  // also refreshes the swing and shift delay tables
    if (clockRunning) clockTimer.update(pulseIntervalMicrosF);
    displayNeedsUpdate = true;
  }
}


// ============================================================================
//  LED UPDATES
//  Call from loop() — NOT from ISR.
//  leds.show() takes ~300µs and must not be called inside the timer interrupt.
//  Each port's LED blinks at its own rate, matching that port's clock division.
// ============================================================================

void updateAllLEDs() {
  uint32_t now     = millis();
  bool     changed = false;

  for (int i = 0; i < 8; i++) {
    uint32_t color;

    if (!ports[i].enabled) {
      color = leds.Color(0, 0, 0);  // off when port is disabled

    } else if (now - ledFlashTime[i] < LED_FLASH_DURATION) {
      color = leds.Color(255, 255, 255);  // bright white flash on pulse

    } else {
      // Idle glow: blue = MIDI port, green = CV port
      color = (ports[i].type == PORT_MIDI)
              ? leds.Color(0, 0, 40)   // dim blue
              : leds.Color(0, 40, 0);  // dim green
    }

    if (leds.getPixelColor(i) != color) {
      leds.setPixelColor(i, color);
      changed = true;
    }
  }

  if (changed) leds.show();
}


// ============================================================================
//  OLED DISPLAY
//
//  Three screens depending on currentLevel:
//    LEVEL_PORT_SELECT  — overview of all 8 ports + BPM
//    LEVEL_PARAM_SELECT — parameter list for selected port
//    LEVEL_VALUE_EDIT   — single parameter editing view
// ============================================================================

// Print a shift value in ms with an explicit sign. Exactly 0 prints as a
// plain "0" (no sign, no decimals) so an on-grid port is obvious at a glance.
// (shift is stored in 0.5ms steps; print() supplies the "-" for negatives.)
void printShiftValue(int8_t shiftHalfMs) {
  if (shiftHalfMs == 0) {
    display.print("0");
    return;
  }
  if (shiftHalfMs > 0) display.print("+");
  display.print(shiftHalfMs * 0.5f, 1);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  float effectiveBPM = bpm * speedMultiplier;

  // ── BPM header (always shown) ─────────────────────────────────────────────
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(effectiveBPM, 1);
  display.print(" BPM");

  // Running/stopped indicator
  display.setTextSize(1);
  display.setCursor(0, 18);
  if (clockRunning) {
    display.print(">>> RUNNING");
  } else {
    display.print("--- STOPPED");
  }

  // Speed multiplier indicator (only shown when not at 1×)
  if (speedMultiplier == 0.5f)      display.print(" x0.5");
  else if (speedMultiplier == 2.0f) display.print(" x2");

  display.drawLine(0, 27, 127, 27, SH110X_WHITE);  // separator line

  // ── Level-specific content ─────────────────────────────────────────────────

  if (currentLevel == LEVEL_PORT_SELECT) {
    // Compact overview: 4 ports per row, 2 rows
    display.setTextSize(1);
    for (int i = 0; i < 8; i++) {
      int col = (i % 4) * 32;
      int row = (i / 4) * 10 + 30;
      display.setCursor(col, row);
      // ">" = selected; "*" = port has a nonzero time shift (off the grid)
      if (i == selectedPort)          display.print(">");
      else if (ports[i].shift != 0)   display.print("*");
      else                            display.print(" ");
      display.print(i + 1);
      display.print(":");
      display.print(ports[i].type == PORT_MIDI ? "MI" : "CV");
    }

    // Selected port detail at bottom — shows the time shift when nonzero
    // (a stray shift should never be invisible), swing otherwise.
    display.setCursor(0, 54);
    display.print("P");
    display.print(selectedPort + 1);
    display.print(": ");
    display.print(divisionNames[ports[selectedPort].division]);
    if (ports[selectedPort].shift != 0) {
      display.print(" TS:");
      printShiftValue(ports[selectedPort].shift);
    } else {
      display.print(" SW:");
      display.print(ports[selectedPort].swing);
      display.print("%");
    }

  } else if (currentLevel == LEVEL_PARAM_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 29);
    display.print("PORT ");
    display.print(selectedPort + 1);
    display.print(" - ");
    display.print(ports[selectedPort].type == PORT_MIDI ? "MIDI" : "CV  ");

    // Five parameters no longer fit under the header on the 64px display —
    // show a 3-row window that scrolls with the selection.
    int firstRow = (int)selectedParam - 1;
    if (firstRow < 0) firstRow = 0;
    if (firstRow > PARAM_COUNT - 3) firstRow = PARAM_COUNT - 3;

    for (int row = 0; row < 3; row++) {
      int i = firstRow + row;
      display.setCursor(0, 40 + row * 8);
      display.print(i == (int)selectedParam ? "> " : "  ");
      display.print(paramNames[i]);
      display.print(": ");

      switch (i) {
        case PARAM_TYPE:
          display.print(ports[selectedPort].type == PORT_MIDI ? "MIDI" : "CV");
          break;
        case PARAM_DIVISION:
          display.print(divisionNames[ports[selectedPort].division]);
          break;
        case PARAM_SHIFT:
          printShiftValue(ports[selectedPort].shift);
          if (ports[selectedPort].shift != 0) display.print("ms");
          break;
        case PARAM_SWING:
          display.print(ports[selectedPort].swing);
          display.print("%");
          break;
        case PARAM_ENABLED:
          display.print(ports[selectedPort].enabled ? "YES" : "NO");
          break;
      }
    }

  } else if (currentLevel == LEVEL_VALUE_EDIT) {
    display.setTextSize(1);
    display.setCursor(0, 29);
    display.print("PORT ");
    display.print(selectedPort + 1);

    display.print(" - ");
    display.print(paramNames[selectedParam]);

    // Division and Swing are not editable on MIDI ports — show the value in
    // square brackets (instead of the < > "adjustable" arrows) plus a hint
    // line saying why.
    bool valueLocked = (ports[selectedPort].type == PORT_MIDI &&
                        (selectedParam == PARAM_DIVISION ||
                         selectedParam == PARAM_SWING));

    if (valueLocked) {
      display.setTextSize(2);
      display.setCursor(0, 38);
      display.print("[");
      if (selectedParam == PARAM_DIVISION) {
        display.print(divisionNames[ports[selectedPort].division]);
      } else {
        display.print(ports[selectedPort].swing);
        display.print("%");
      }
      display.print("]");
      display.setTextSize(1);
      display.setCursor(0, 56);
      display.print(selectedParam == PARAM_DIVISION
                    ? "LOCKED - MIDI 24 PPQN"
                    : "LOCKED - CV ports only");

    } else if (selectedParam == PARAM_SHIFT) {
      // Shift gets its own layout: value raised to make room for a hint line.
      display.setTextSize(2);
      display.setCursor(10, 38);
      display.print("< ");
      printShiftValue(ports[selectedPort].shift);
      display.print(" >");
      display.setTextSize(1);
      display.setCursor(0, 56);
      display.print("ms  (long-press = 0)");

    } else {
      // Show current value large, centred, with < > arrows
      display.setTextSize(2);
      display.setCursor(10, 42);
      display.print("< ");

      switch (selectedParam) {
        case PARAM_TYPE:
          display.print(ports[selectedPort].type == PORT_MIDI ? "MIDI" : "CV  ");
          break;
        case PARAM_DIVISION:
          display.print(divisionNames[ports[selectedPort].division]);
          break;
        case PARAM_SWING:
          display.print(ports[selectedPort].swing);
          display.print("%");
          break;
        case PARAM_ENABLED:
          display.print(ports[selectedPort].enabled ? "YES" : "NO ");
          break;

        default:
          break;
      }

      display.print(" >");
    }
  }

  display.display();
  displayNeedsUpdate = false;
}


// ============================================================================
//  SETUP
// ============================================================================

void setup() {
  // Initialise MIDI output UARTs
  initMIDIPorts();

  // Initialise CV output GPIO pins
  initCVPins();

  // Input pins with internal pull-up resistors
  pinMode(BTN_START_STOP, INPUT_PULLUP);
  pinMode(NAV_BTN_PIN,    INPUT_PULLUP);
  pinMode(SW_HALF,        INPUT_PULLUP);
  pinMode(SW_DOUBLE,      INPUT_PULLUP);

  // Start/Stop button bicolour LED — PWM output pins
  pinMode(BTN_LED_GREEN, OUTPUT);
  pinMode(BTN_LED_RED,   OUTPUT);
  analogWrite(BTN_LED_GREEN, 0);
  analogWrite(BTN_LED_RED,   BTN_LED_RED_IDLE);  // start in stopped state
  btnLedCycleStart = micros();

  // Calculate initial timing values
  updateBPM(120.0f);

  // Initialise NeoPixel LEDs
  leds.begin();
  leds.setBrightness(80);  // 0–255; don't run at full brightness
  leds.clear();
  leds.show();

  // Initialise OLED display. If it isn't found the sketch continues anyway —
  // LEDs and MIDI still work.
  display.begin(0x3C, true);
  display.clearDisplay();
  display.display();
  updateDisplay();

  // Brief startup LED sequence: all white, then settle to idle colours
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, leds.Color(255, 255, 255));
  }
  leds.show();
  delay(300);
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, ports[i].type == PORT_MIDI
                          ? leds.Color(0, 0, 40)
                          : leds.Color(0, 40, 0));
  }
  leds.show();
}


// ============================================================================
//  MAIN LOOP
//
//  Keep everything here non-blocking (no delay() calls).
//  The clock ISR fires independently via hardware timer at highest priority.
//  This loop handles UI, LEDs, and CV pulse timing.
// ============================================================================

void loop() {
  handleTempoEncoder();      // Encoder 1 — BPM adjustment
  handleNavEncoder();        // Encoder 2 — port/param navigation
  handleNavButton();         // Encoder 2 push — navigate in/out of menus
  handleStartStopButton();   // Start / Stop / Resync
  handleSpeedSwitch();       // Half / Normal / Double speed toggle
  updateCVPulseOff();        // Turn off CV pulses whose time has elapsed
  updateScheduledPulses();   // Fire time-shifted / swing-delayed pulses
  updateUSBMIDI();           // Send ISR-counted clock ticks to USB MIDI
  updateAllLEDs();           // Update NeoPixel LEDs (each port at its own rate)
  updateButtonLED();         // Breathe start/stop button LED in sync with BPM

  if (displayNeedsUpdate) {
    updateDisplay();         // Refresh OLED only when something has changed
  }
}


// ============================================================================
//  END OF CLOCKTOPUS SKETCH
//
//  First-time upload steps:
//    1. Install Teensyduino from pjrc.com
//    2. Open this file in Arduino IDE
//    3. Tools > Board > Teensyduino > Teensy 4.1
//    4. Tools > USB Type > Serial + MIDI
//    5. Connect Teensy via USB and click Upload
//
//  Suggested first test (no hardware needed beyond Teensy + USB):
//    - Upload sketch
//    - Open a DAW or MIDI monitor app on your computer
//    - Press the start button — MIDI clock messages should arrive on
//      USB MIDI at the set BPM
// ============================================================================
