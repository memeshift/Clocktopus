// ============================================================================
//  CLOCKTOPUS
//  8-Port MIDI / CV Eurorack Clock Sync Device
//  Platform: Teensy 4.1 (600MHz ARM Cortex-M7)
//
//  Hardware:
//    - 8x MIDI outputs via hardware UARTs (Serial1–Serial8)
//    - 8x Eurorack CV outputs via GPIO + 74AHCT125 level shifter
//    - 8x WS2812B NeoPixel LEDs (one per port)
//    - Encoder 1 (pins 14,15) — Tempo
//    - Encoder 2 (pins 18,19) — Port config navigation
//    - Nav encoder button (pin 23)
//    - Start/Stop button (pin 20)
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

// Double-press window for resync (milliseconds)
const uint16_t DOUBLE_PRESS_MS = 400;

// Long press threshold for nav encoder (milliseconds)
const uint16_t LONG_PRESS_MS = 600;

// Internal resolution: 96 PPQN gives clean division for all musical values
// (96 divides evenly into whole/half/quarter/8th/16th/32nd notes)
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
  PortType type;       // MIDI or CV
  uint8_t  division;  // Index into divisionNames[] / divisionTicks[]
  uint8_t  swing;     // Swing percentage: 50 = straight, 90 = heavy swing
  bool     enabled;   // Whether this port is active
};


// ============================================================================
//  CLOCK DIVISION TABLES
// ============================================================================
//
//  divisionTicks[] = how many internal ticks (at 96 PPQN) between CV pulses
//  divisionNames[] = human-readable label shown on OLED
//
//  96 ticks = whole note (1 per bar)
//  48 ticks = half note
//  24 ticks = quarter note   (standard "1 pulse per beat")
//  12 ticks = 8th note
//   6 ticks = 16th note
//   3 ticks = 32nd note
//   1 tick  = raw MIDI clock rate (24 pulses per quarter note)
//   6 ticks = Volca sync (same as 16th note — Korg Volca expects this rate)

const uint8_t NUM_DIVISIONS = 8;

const char* divisionNames[NUM_DIVISIONS] = {
  "Whole",      // index 0
  "Half",       // index 1
  "Quarter",    // index 2
  "8th",        // index 3
  "16th",       // index 4
  "32nd",       // index 5
  "MIDI(24)",   // index 6 — raw 24 PPQN MIDI clock rate
  "Volca"       // index 7 — Korg Volca sync (16th note pulse rate)
};

const uint8_t divisionTicks[NUM_DIVISIONS] = {
  96,  // whole note
  48,  // half note
  24,  // quarter note
  12,  // 8th note
   6,  // 16th note
   3,  // 32nd note
   4,  // MIDI 24PPQN (96/24 = 4 ticks per MIDI clock pulse)
   6   // Volca (same as 16th note)
};


// ============================================================================
//  GLOBAL STATE — CLOCK
// ============================================================================

volatile float    bpm                  = 120.0f;
volatile uint32_t pulseIntervalMicros  = 20833;  // recalculated by updateBPM()
volatile bool     clockRunning         = false;
volatile uint32_t internalTickCount    = 0;       // 0–95, wraps at INTERNAL_PPQN

float   speedMultiplier = 1.0f;   // 0.5 / 1.0 / 2.0 depending on toggle switch


// ============================================================================
//  GLOBAL STATE — PORTS
// ============================================================================

// Default configuration: all ports start as CV, quarter-note division,
// no swing, enabled. Change these defaults to suit your setup.
PortConfig ports[8] = {
  {PORT_MIDI, 2, 50, true},   // port 1: MIDI, quarter note
  {PORT_CV,   2, 50, true},   // port 2: CV,   quarter note
  {PORT_CV,   3, 50, true},   // port 3: CV,   8th note
  {PORT_CV,   4, 50, true},   // port 4: CV,   16th note
  {PORT_CV,   1, 50, true},   // port 5: CV,   half note
  {PORT_CV,   0, 50, true},   // port 6: CV,   whole note
  {PORT_CV,   2, 50, true},   // port 7: CV,   quarter note
  {PORT_CV,   2, 50, true},   // port 8: CV,   quarter note
};

// Per-port tick counters for clock division
uint8_t  portTickCount[8]   = {0};

// Per-port CV pulse-off times (micros timestamp when pulse should end)
uint32_t cvPulseOffTime[8]  = {0};

// Per-port MIDI clock division counters
// (for MIDI ports, we count internal ticks to know when to send a MIDI clock byte)
uint8_t  midiTickCount[8]   = {0};

// Per-port swing: 8th-note phase tracker (0=downbeat, 1=upbeat)
uint8_t  eighthNotePhase[8] = {0};

// Pre-computed swing delay per port in microseconds
uint32_t swingDelayMicros[8] = {0};

// Per-port delayed CV pulse trigger times (for swing on upbeats)
uint32_t cvSwingTriggerTime[8] = {0};


// ============================================================================
//  GLOBAL STATE — UI
// ============================================================================

UILevel   currentLevel  = LEVEL_PORT_SELECT;
uint8_t   selectedPort  = 0;        // 0–7
PortParam selectedParam = PARAM_TYPE;
bool      displayNeedsUpdate = true;


// ============================================================================
//  GLOBAL STATE — LEDS
// ============================================================================

uint32_t ledFlashTime[8] = {0};  // millis() timestamp of last pulse per port


// ============================================================================
//  GLOBAL STATE — BUTTONS
// ============================================================================

// Start/Stop button
bool     btnLastState   = HIGH;
uint32_t lastPressTime  = 0;

// Nav encoder button
bool     navLastState   = HIGH;
uint32_t navPressTime   = 0;
bool     navButtonHeld  = false;

// Tap tempo
uint32_t tapTimes[4]    = {0};
uint8_t  tapIndex       = 0;


// ============================================================================
//  HARDWARE OBJECTS
// ============================================================================

IntervalTimer clockTimer;

Encoder tempoEncoder(14, 15);   // Encoder 1: Tempo
Encoder navEncoder(18, 19);     // Encoder 2: Port config navigation

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

// Call whenever BPM or speedMultiplier changes
void updateBPM(float newBPM) {
  bpm = constrain(newBPM, 20.0f, 300.0f);
  // Internal tick interval in microseconds
  // = 60,000,000 µs/min  ÷  (BPM × speedMultiplier × INTERNAL_PPQN)
  pulseIntervalMicros = (uint32_t)(60000000.0f /
                        (bpm * speedMultiplier * (float)INTERNAL_PPQN));
}

// Recompute swing delay for every port
// Call after any BPM change or after editing a port's swing value
void updateSwingDelays() {
  // Duration of one 8th note in microseconds at current tempo
  uint32_t eighthNoteMicros = pulseIntervalMicros * (INTERNAL_PPQN / 8);
  for (int i = 0; i < 8; i++) {
    // swingPercent 50 = straight (0 delay on upbeat)
    // swingPercent 75 = strong swing (delay = 25% of 8th note duration)
    // swingPercent 90 = very heavy swing
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

// Send a byte to a single MIDI port (only if it's configured as MIDI and enabled)
inline void sendMIDI(uint8_t portIndex, uint8_t msg) {
  if (ports[portIndex].enabled && ports[portIndex].type == PORT_MIDI) {
    midiPorts[portIndex]->write(msg);
  }
}

// Send a byte to ALL enabled MIDI ports (used for START/STOP/CONTINUE)
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

// Called from main loop — turns off any CV pulses whose time has elapsed
void updateCVPulseOff() {
  uint32_t now = micros();
  for (int i = 0; i < 8; i++) {
    if (cvPulseOffTime[i] > 0 && now >= cvPulseOffTime[i]) {
      digitalWriteFast(CV_PINS[i], LOW);
      cvPulseOffTime[i] = 0;
    }
  }
}

// Called from main loop — fires any CV pulses that have been swing-delayed
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

      // Determine if this is an upbeat (odd 8th note) for swing
      bool isUpbeat = (eighthNotePhase[i] == 1);

      // Toggle 8th note phase
      // An 8th note = INTERNAL_PPQN/8 = 12 ticks at 96 PPQN
      // We track this separately per port based on their own division firing
      eighthNotePhase[i] ^= 1;

      if (ports[i].type == PORT_MIDI) {
        // Send MIDI clock byte immediately
        midiPorts[i]->write(MIDI_CLOCK);
        ledFlashTime[i] = millis();  // schedule LED flash (millis safe in ISR on Teensy)

      } else {
        // CV output
        if (isUpbeat && swingDelayMicros[i] > 0) {
          // Delay this upbeat pulse for swing effect
          cvSwingTriggerTime[i] = now + swingDelayMicros[i];
          // LED will flash when the delayed trigger fires
        } else {
          // Fire immediately
          digitalWriteFast(CV_PINS[i], HIGH);
          cvPulseOffTime[i] = now + CV_PULSE_WIDTH_US;
          ledFlashTime[i]   = millis();
        }
      }
    }
  }

  // Advance global tick counter
  internalTickCount = (internalTickCount + 1) % INTERNAL_PPQN;
}


// ============================================================================
//  CLOCK TRANSPORT
// ============================================================================

void startClock() {
  if (clockRunning) return;

  // Reset all counters for a clean start from beat 1
  internalTickCount = 0;
  for (int i = 0; i < 8; i++) {
    portTickCount[i]      = 0;
    midiTickCount[i]      = 0;
    eighthNotePhase[i]    = 0;
    cvSwingTriggerTime[i] = 0;
    cvPulseOffTime[i]     = 0;
  }

  clockRunning = true;
  sendToAllMIDIPorts(MIDI_START);
  clockTimer.begin(clockISR, pulseIntervalMicros);

  displayNeedsUpdate = true;
}

void stopClock() {
  if (!clockRunning) return;

  clockRunning = false;
  clockTimer.end();
  sendToAllMIDIPorts(MIDI_STOP);

  // Kill all CV outputs immediately
  for (int i = 0; i < 8; i++) {
    digitalWriteFast(CV_PINS[i], LOW);
    cvPulseOffTime[i]     = 0;
    cvSwingTriggerTime[i] = 0;
  }

  displayNeedsUpdate = true;
}

void resyncClock() {
  // Atomically restart the clock from beat 1 without sending STOP/START
  clockTimer.end();

  internalTickCount = 0;
  for (int i = 0; i < 8; i++) {
    portTickCount[i]      = 0;
    midiTickCount[i]      = 0;
    eighthNotePhase[i]    = 0;
    cvSwingTriggerTime[i] = 0;
  }

  clockTimer.begin(clockISR, pulseIntervalMicros);
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

    // Hold the nav encoder button while turning tempo encoder for coarse adjust
    // (5 BPM per step instead of 0.5 BPM)
    float increment = (digitalRead(NAV_BTN_PIN) == LOW) ? 5.0f : 0.5f;
    updateBPM(bpm + (steps * increment));
    updateSwingDelays();

    if (clockRunning) {
      clockTimer.update(pulseIntervalMicros);  // update without restarting
    }

    displayNeedsUpdate = true;
  }
}


// ============================================================================
//  ENCODER 2 — PORT CONFIG NAVIGATION
// ============================================================================

// Edit the currently selected parameter value by `steps` steps
void editCurrentValue(int steps) {
  PortConfig& p = ports[selectedPort];

  switch (selectedParam) {

    case PARAM_TYPE:
      // Toggle between MIDI and CV
      p.type = (p.type == PORT_MIDI) ? PORT_CV : PORT_MIDI;

      if (p.type == PORT_MIDI) {
        // Switching TO MIDI: set up UART, disable CV pin
        midiPorts[selectedPort]->begin(31250);
        digitalWriteFast(CV_PINS[selectedPort], LOW);
        pinMode(CV_PINS[selectedPort], INPUT);  // stop driving the pin
      } else {
        // Switching TO CV: set pin as output
        pinMode(CV_PINS[selectedPort], OUTPUT);
        digitalWriteFast(CV_PINS[selectedPort], LOW);
      }
      break;

    case PARAM_DIVISION:
      p.division = (uint8_t)constrain((int)p.division + steps, 0, NUM_DIVISIONS - 1);
      // Reset tick counter so new division takes effect cleanly
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

  // Button just pressed — record time
  if (pressed && !navButtonHeld) {
    navPressTime   = now;
    navButtonHeld  = true;
    navLastState   = LOW;
  }

  // Button just released
  if (!pressed && navButtonHeld) {
    uint32_t heldFor = now - navPressTime;
    navButtonHeld    = false;
    navLastState     = HIGH;

    if (heldFor >= LONG_PRESS_MS) {
      // Long press — return to top level from anywhere
      currentLevel = LEVEL_PORT_SELECT;
    } else {
      // Short press — advance deeper or confirm
      switch (currentLevel) {
        case LEVEL_PORT_SELECT:
          currentLevel  = LEVEL_PARAM_SELECT;
          selectedParam = PARAM_TYPE;
          break;

        case LEVEL_PARAM_SELECT:
          currentLevel = LEVEL_VALUE_EDIT;
          break;

        case LEVEL_VALUE_EDIT:
          // Confirm current value and step back up to param list
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
      // Double press = RESYNC
      if (clockRunning) resyncClock();
    } else {
      // Single press = toggle start/stop
      if (clockRunning) stopClock();
      else              startClock();
    }
    lastPressTime = now;
  }

  btnLastState = state;
}


// ============================================================================
//  TAP TEMPO
//  Connect a dedicated tap button and call this from loop() when it's pressed.
//  Averages the last 3 tap intervals for stability.
//  (Optional — wire to a second button or use a long-press on start/stop button)
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
//  Wire centre pin to GND; two outer pins to SW_HALF and SW_DOUBLE with INPUT_PULLUP
// ============================================================================

void handleSpeedSwitch() {
  float newMult = 1.0f;

  if      (digitalRead(SW_HALF)   == LOW) newMult = 0.5f;
  else if (digitalRead(SW_DOUBLE) == LOW) newMult = 2.0f;

  if (newMult != speedMultiplier) {
    speedMultiplier = newMult;
    updateBPM(bpm);  // recalculates pulseIntervalMicros with new multiplier
    updateSwingDelays();
    if (clockRunning) clockTimer.update(pulseIntervalMicros);
    displayNeedsUpdate = true;
  }
}


// ============================================================================
//  LED UPDATES
//  Call from loop() — NOT from ISR.
//  leds.show() takes ~300µs and must not be called inside the timer interrupt.
// ============================================================================

void updateAllLEDs() {
  uint32_t now     = millis();
  bool     changed = false;

  for (int i = 0; i < 8; i++) {
    uint32_t color;

    if (!ports[i].enabled) {
      // Port disabled — off
      color = leds.Color(0, 0, 0);

    } else if (now - ledFlashTime[i] < LED_FLASH_DURATION) {
      // Recently pulsed — bright white flash
      color = leds.Color(255, 255, 255);

    } else {
      // Idle glow — blue for MIDI, green for CV
      if (ports[i].type == PORT_MIDI)
        color = leds.Color(0, 0, 40);   // dim blue
      else
        color = leds.Color(0, 40, 0);   // dim green
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

  // Speed multiplier indicator
  if (speedMultiplier == 0.5f)     display.print(" x0.5");
  else if (speedMultiplier == 2.0f) display.print(" x2");

  display.drawLine(0, 27, 127, 27, SSD1306_WHITE);  // separator

  // ── Level-specific content ─────────────────────────────────────────────────

  if (currentLevel == LEVEL_PORT_SELECT) {
    // Show all 8 ports as abbreviated summary: "1:MI" or "1:CV"
    display.setTextSize(1);
    for (int i = 0; i < 8; i++) {
      int col = (i % 4) * 32;   // 4 ports per row, 32px wide each
      int row = (i / 4) * 10 + 30;
      display.setCursor(col, row);
      // Highlight selected port with an arrow
      if (i == selectedPort) display.print(">");
      else                   display.print(" ");
      display.print(i + 1);
      display.print(":");
      display.print(ports[i].type == PORT_MIDI ? "MI" : "CV");
    }

    // Show selected port's division at bottom
    display.setCursor(0, 54);
    display.print("P");
    display.print(selectedPort + 1);
    display.print(": ");
    display.print(divisionNames[ports[selectedPort].division]);
    display.print(" SW:");
    display.print(ports[selectedPort].swing);
    display.print("%");

  } else if (currentLevel == LEVEL_PARAM_SELECT) {
    // Show parameter list for selected port
    display.setTextSize(1);
    display.setCursor(0, 29);
    display.print("PORT ");
    display.print(selectedPort + 1);
    display.print(" - ");
    display.print(ports[selectedPort].type == PORT_MIDI ? "MIDI" : "CV  ");

    const char* paramNames[PARAM_COUNT] = {"Type", "Division", "Swing", "Enabled"};

    for (int i = 0; i < PARAM_COUNT; i++) {
      display.setCursor(0, 38 + i * 8);
      display.print(i == selectedParam ? "> " : "  ");
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
    // Show single-parameter edit view
    display.setTextSize(1);
    display.setCursor(0, 29);
    display.print("PORT ");
    display.print(selectedPort + 1);

    const char* paramNames[PARAM_COUNT] = {"Type", "Division", "Swing", "Enabled"};
    display.print(" - ");
    display.print(paramNames[selectedParam]);

    // Show current value large and centred
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

  // Calculate initial timing values
  updateBPM(120.0f);
  updateSwingDelays();

  // Initialise NeoPixel LEDs
  leds.begin();
  leds.setBrightness(80);  // 0-255, don't run at full brightness
  leds.clear();
  leds.show();

  // Initialise OLED display
  // If display.begin() fails, the sketch continues but the screen stays blank
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // Display not found — continue anyway, LEDs and MIDI still work
  }
  display.clearDisplay();
  display.display();

  // Draw initial display
  updateDisplay();

  // Brief startup LED sequence — all lights white then settle to idle colour
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, leds.Color(255, 255, 255));
  }
  leds.show();
  delay(300);
  for (int i = 0; i < 8; i++) {
    leds.setPixelColor(i, ports[i].type == PORT_MIDI ?
                       leds.Color(0, 0, 40) : leds.Color(0, 40, 0));
  }
  leds.show();
}


// ============================================================================
//  MAIN LOOP
//
//  Keep everything here non-blocking (no delay() calls).
//  The clock ISR fires independently via hardware timer.
//  This loop handles UI, LEDs, and CV pulse management.
// ============================================================================

void loop() {
  handleTempoEncoder();      // Encoder 1 — BPM adjustment
  handleNavEncoder();        // Encoder 2 — port/param navigation
  handleNavButton();         // Encoder 2 push — navigate in/out of menus
  handleStartStopButton();   // Start / Stop / Resync
  handleSpeedSwitch();       // Half / Normal / Double speed toggle
  updateCVPulseOff();        // Turn off CV pulses whose time has elapsed
  updateCVSwingTriggers();   // Fire any swing-delayed CV pulses
  updateAllLEDs();           // Update NeoPixel LEDs (per-port, independent rhythm)

  if (displayNeedsUpdate) {
    updateDisplay();         // Refresh OLED only when something changed
  }
}


// ============================================================================
//  END OF CLOCKTOPUS SKETCH
//
//  Next steps:
//    1. Install Teensyduino from pjrc.com
//    2. Open this file in Arduino IDE
//    3. Tools > Board > Teensyduino > Teensy 4.1
//    4. Tools > USB Type > Serial + MIDI
//    5. Connect Teensy via USB and click Upload
//
//  Suggested first test (no hardware needed beyond Teensy + USB):
//    - Upload sketch
//    - Open a DAW or MIDI monitor app on your computer
//    - Press start button — you should see MIDI clock messages arriving
//      on USB MIDI at your set BPM
// ============================================================================
