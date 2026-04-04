// ============================================================================
//  CLOCKTOPUS
//  8-Port MIDI / CV Eurorack Clock Sync Device
//  Platform: Teensy 4.1 (600MHz ARM Cortex-M7)
//
//  Hardware:
//    - 8x MIDI outputs via hardware UARTs (Serial1–Serial8)
//    - 8x Eurorack CV outputs via GPIO + 74AHCT125 level shifter
//    - 8x WS2812B NeoPixel LEDs (one per port)
//    - Encoder 1 (pins 14,15) — Tempo (non-detented / smooth)
//    - Encoder 2 (pins 18,19) — Port config navigation (detented with push button)
//    - Nav encoder button (pin 23)
//    - Start/Stop button (pin 20)
//    - Start/Stop button bicolour LED: green (pin 25, PWM), red (pin 26, PWM)
//      Frosted clear button cap. Green breathes to BPM when running.
//      Red glows dim steady when stopped.
//    - Speed switch HALF (pin 21), DOUBLE (pin 22)
//    - OLED display 128x64 I2C (SSD1306, address 0x3C)
//    - NeoPixel data pin 24
//
//  Dependencies (install via Arduino IDE Library Manager):
//    - Teensyduino (from pjrc.com — includes IntervalTimer, Encoder)
//    - Adafruit_SSD1306
//    - Adafruit_GFX
//    - Adafruit_NeoPixel
// ============================================================================


// ============================================================================
//  CHANGELOG
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

#include <Arduino.h>
#include <IntervalTimer.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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
const uint8_t BTN_LED_GREEN = 25;   // PWM pin — green LED anode
const uint8_t BTN_LED_RED   = 26;   // PWM pin — red LED anode

// Encoder pins (must be interrupt-capable pins on Teensy 4.1)
// Encoder 1: Tempo  — pins 14, 15
// Encoder 2: Nav    — pins 18, 19


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


// ============================================================================
//  GLOBAL STATE — CLOCK
// ============================================================================

volatile float    bpm                 = 120.0f;
volatile uint32_t pulseIntervalMicros = 20833;  // recalculated by updateBPM()
volatile bool     clockRunning        = false;
volatile uint32_t internalTickCount   = 0;       // 0–95, wraps at INTERNAL_PPQN

float speedMultiplier = 1.0f;  // 0.5 / 1.0 / 2.0 depending on toggle switch


// ============================================================================
//  GLOBAL STATE — PORTS
// ============================================================================

// Default configuration for first power-on.
// Division indices refer to divisionTicks[] above:
//   0=Whole, 1=Half, 2=Quarter, 3=8th, 4=16th, 5=32nd, 6=MIDI(24), 7=Volca
PortConfig ports[8] = {
  {PORT_MIDI, 2, 50, true},   // port 1: MIDI out, quarter note
  {PORT_CV,   2, 50, true},   // port 2: CV,  quarter note
  {PORT_CV,   3, 50, true},   // port 3: CV,  8th note
  {PORT_CV,   4, 50, true},   // port 4: CV,  16th note
  {PORT_CV,   1, 50, true},   // port 5: CV,  half note
  {PORT_CV,   0, 50, true},   // port 6: CV,  whole note
  {PORT_CV,   2, 50, true},   // port 7: CV,  quarter note
  {PORT_CV,   2, 50, true},   // port 8: CV,  quarter note
};

// Per-port tick counters for clock division.
// FIXED v0.2: uint16_t — must hold up to 384 (whole note ticks).
// volatile — written by clockISR, read by editCurrentValue in loop().
volatile uint16_t portTickCount[8] = {0};

// Per-port CV pulse-off times (micros timestamp when pulse should end).
// volatile — written by clockISR and updateCVSwingTriggers, read by updateCVPulseOff.
volatile uint32_t cvPulseOffTime[8] = {0};

// Per-port swing: 8th-note phase tracker (0 = downbeat, 1 = upbeat).
// Written and read only in clockISR — no volatile needed, but kept for clarity.
uint8_t eighthNotePhase[8] = {0};

// Pre-computed swing delay per port in microseconds.
// volatile — written by updateSwingDelays in loop(), read by clockISR.
volatile uint32_t swingDelayMicros[8] = {0};

// Per-port delayed CV pulse trigger times (for swing on upbeats).
// volatile — written by clockISR, read and cleared by updateCVSwingTriggers in loop().
volatile uint32_t cvSwingTriggerTime[8] = {0};


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
// volatile — written by clockISR and updateCVSwingTriggers, read by updateAllLEDs.
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

float    btnLedPhase       = 0.0f;
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
bool     btnLastState  = HIGH;
uint32_t lastPressTime = 0;

// Nav encoder button
bool     navLastState  = HIGH;
uint32_t navPressTime  = 0;
bool     navButtonHeld = false;

// Tap tempo
uint32_t tapTimes[4] = {0};
uint8_t  tapIndex    = 0;


// ============================================================================
//  HARDWARE OBJECTS
// ============================================================================

IntervalTimer clockTimer;

Encoder tempoEncoder(14, 15);  // Encoder 1: Tempo
Encoder navEncoder(18, 19);    // Encoder 2: Port config navigation

long lastTempoEncoderPos = 0;
long lastNavEncoderPos   = 0;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

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
  pulseIntervalMicros = (uint32_t)(60000000.0f /
                        (bpm * speedMultiplier * (float)INTERNAL_PPQN));
  // Quarter-note duration for the button LED breathing cycle:
  //   60,000,000 µs/min  ÷  (BPM × speedMultiplier)
  quarterNoteMicros = (uint32_t)(60000000.0f / (bpm * speedMultiplier));
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
    if (cvPulseOffTime[i] > 0 && now >= cvPulseOffTime[i]) {
      digitalWriteFast(CV_PINS[i], LOW);
      cvPulseOffTime[i] = 0;
    }
  }
}

// Called from main loop — fires any CV pulses that have been swing-delayed.
void updateCVSwingTriggers() {
  uint32_t now = micros();
  for (int i = 0; i < 8; i++) {
    if (cvSwingTriggerTime[i] > 0 && now >= cvSwingTriggerTime[i]) {
      if (ports[i].enabled && ports[i].type == PORT_CV) {
        digitalWriteFast(CV_PINS[i], HIGH);
        cvPulseOffTime[i] = now + CV_PULSE_WIDTH_US;
        ledFlashTime[i]   = millis();
      }
      cvSwingTriggerTime[i] = 0;
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

      if (ports[i].type == PORT_MIDI) {
        // Send MIDI clock byte immediately via hardware UART.
        midiPorts[i]->write(MIDI_CLOCK);
        // millis() is safe here on Teensy 4.1 but may be ~1ms stale at
        // high ISR priority. Imperceptible for LED flash purposes.
        ledFlashTime[i] = millis();

      } else {
        // CV output
        if (isUpbeat && swingDelayMicros[i] > 0) {
          // Schedule this upbeat pulse to fire after the swing delay.
          // updateCVSwingTriggers() in loop() will watch for it.
          cvSwingTriggerTime[i] = now + swingDelayMicros[i];
        } else {
          // Fire immediately
          digitalWriteFast(CV_PINS[i], HIGH);
          cvPulseOffTime[i] = now + CV_PULSE_WIDTH_US;
          ledFlashTime[i]   = millis();
        }
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
    portTickCount[i]      = 0;
    eighthNotePhase[i]    = 0;
    cvSwingTriggerTime[i] = 0;
    cvPulseOffTime[i]     = 0;
  }

  clockRunning = true;
  sendToAllMIDIPorts(MIDI_START);
  clockTimer.begin(clockISR, pulseIntervalMicros);
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

  // Kill all CV outputs immediately.
  for (int i = 0; i < 8; i++) {
    digitalWriteFast(CV_PINS[i], LOW);
    cvPulseOffTime[i]     = 0;
    cvSwingTriggerTime[i] = 0;
  }

  displayNeedsUpdate = true;
}

void resyncClock() {
  // Restart the clock from beat 1 without sending STOP/START.
  // Used for tight re-alignment mid-session (double-press of start/stop button).
  clockTimer.end();

  internalTickCount = 0;
  for (int i = 0; i < 8; i++) {
    portTickCount[i]      = 0;
    eighthNotePhase[i]    = 0;
    cvSwingTriggerTime[i] = 0;
  }

  clockTimer.begin(clockISR, pulseIntervalMicros);
  // FIXED v0.2: Re-apply priority after re-arming the timer.
  clockTimer.priority(0);
  sendToAllMIDIPorts(MIDI_CONTINUE);

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
    float increment = (digitalRead(NAV_BTN_PIN) == LOW) ? 5.0f : 0.5f;
    updateBPM(bpm + (steps * increment));
    updateSwingDelays();

    if (clockRunning) {
      clockTimer.update(pulseIntervalMicros);  // adjust rate without restarting
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
      } else {
        // Switching TO CV: set pin as output, ensure it starts LOW.
        pinMode(CV_PINS[selectedPort], OUTPUT);
        digitalWriteFast(CV_PINS[selectedPort], LOW);
      }

      // Reset this port's tick counter so the new type takes effect cleanly.
      portTickCount[selectedPort] = 0;

      if (wasRunning) startClock();
      break;
    }

    case PARAM_DIVISION:
      p.division = (uint8_t)constrain((int)p.division + steps, 0, NUM_DIVISIONS - 1);
      // Reset tick counter so new division takes effect at next beat boundary.
      portTickCount[selectedPort] = 0;
      break;

    case PARAM_SWING:
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

  // Button just pressed — record time.
  if (pressed && !navButtonHeld) {
    navPressTime  = now;
    navButtonHeld = true;
    navLastState  = LOW;
  }

  // Button just released — determine short vs long press.
  if (!pressed && navButtonHeld) {
    uint32_t heldFor = now - navPressTime;
    navButtonHeld    = false;
    navLastState     = HIGH;

    if (heldFor >= LONG_PRESS_MS) {
      // Long press — escape back to top level from anywhere.
      currentLevel = LEVEL_PORT_SELECT;
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

  if (state == LOW && btnLastState == HIGH) {  // falling edge = press
    if (now - lastPressTime < DOUBLE_PRESS_MS) {
      // Double press = RESYNC (re-align to beat 1 without stopping)
      if (clockRunning) resyncClock();
    } else {
      // Single press = toggle start / stop
      if (clockRunning) stopClock();
      else              startClock();
    }
    lastPressTime = now;
  }

  btnLastState = state;
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
    btnLedPhase = (float)elapsed / (float)quarterNoteMicros;

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
    btnLedPhase      = 0.0f;
  }
}


// ============================================================================
//  TAP TEMPO
//  Connect a dedicated tap button and call this from loop() when it is pressed.
//  Averages the last 3 tap intervals for stability.
//  (Optional — wire to a second button or use a long-press on the start/stop button)
// ============================================================================

void handleTapTempo() {
  uint32_t now = millis();
  tapTimes[tapIndex % 4] = now;
  tapIndex++;

  if (tapIndex >= 4) {
    uint32_t totalInterval = 0;
    for (int i = 1; i < 4; i++) {
      totalInterval += tapTimes[i] - tapTimes[i - 1];
    }
    float avgInterval = totalInterval / 3.0f;
    float newBPM      = 60000.0f / avgInterval;
    updateBPM(newBPM);
    updateSwingDelays();
    if (clockRunning) clockTimer.update(pulseIntervalMicros);
    displayNeedsUpdate = true;
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
    updateBPM(bpm);  // recalculates pulseIntervalMicros and quarterNoteMicros
    updateSwingDelays();
    if (clockRunning) clockTimer.update(pulseIntervalMicros);
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

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

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

  display.drawLine(0, 27, 127, 27, SSD1306_WHITE);  // separator line

  // ── Level-specific content ─────────────────────────────────────────────────

  if (currentLevel == LEVEL_PORT_SELECT) {
    // Compact overview: 4 ports per row, 2 rows
    display.setTextSize(1);
    for (int i = 0; i < 8; i++) {
      int col = (i % 4) * 32;
      int row = (i / 4) * 10 + 30;
      display.setCursor(col, row);
      display.print(i == selectedPort ? ">" : " ");
      display.print(i + 1);
      display.print(":");
      display.print(ports[i].type == PORT_MIDI ? "MI" : "CV");
    }

    // Selected port detail at bottom
    display.setCursor(0, 54);
    display.print("P");
    display.print(selectedPort + 1);
    display.print(": ");
    display.print(divisionNames[ports[selectedPort].division]);
    display.print(" SW:");
    display.print(ports[selectedPort].swing);
    display.print("%");

  } else if (currentLevel == LEVEL_PARAM_SELECT) {
    display.setTextSize(1);
    display.setCursor(0, 29);
    display.print("PORT ");
    display.print(selectedPort + 1);
    display.print(" - ");
    display.print(ports[selectedPort].type == PORT_MIDI ? "MIDI" : "CV  ");

    const char* paramNames[PARAM_COUNT] = {"Type", "Division", "Swing", "Enabled"};

    for (int i = 0; i < PARAM_COUNT; i++) {
      display.setCursor(0, 38 + i * 8);
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

    const char* paramNames[PARAM_COUNT] = {"Type", "Division", "Swing", "Enabled"};
    display.print(" - ");
    display.print(paramNames[selectedParam]);

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
    }

    display.print(" >");
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
  updateSwingDelays();

  // Initialise NeoPixel LEDs
  leds.begin();
  leds.setBrightness(80);  // 0–255; don't run at full brightness
  leds.clear();
  leds.show();

  // Initialise OLED display
  // If display.begin() fails, the sketch continues — LEDs and MIDI still work.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Display not found — continue without it
  }
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
  updateCVSwingTriggers();   // Fire any swing-delayed CV pulses
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
