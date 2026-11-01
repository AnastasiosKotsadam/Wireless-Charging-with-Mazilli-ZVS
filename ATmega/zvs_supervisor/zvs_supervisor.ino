/*
  ZVS driver supervisor - first version
  -------------------------------------
  Target : ATmega328P, 16 MHz external crystal, 5 V
           Arduino IDE -> Board: MiniCore -> ATmega328, Clock: External 16 MHz, BOD: 4.3 V
           (also runs unchanged on an Arduino Uno / DFRduino / Nano)

  Jobs:
    1. Two current thresholds for the LM393 via PWM on D3:
         - startup (hard) limit while the ZVS starts
         - normal limit once it runs
    2. Oscillation watchdog: counts the ZVS pulses on D5 (Timer1 / T1 pin)
       and trips the latch if the frequency is missing or outside the window
    3. Fault diagnosis by reading the latch output (D8):
         - latch set, but not by the ATmega  -> overcurrent  (RED)
         - latch set by the ATmega            -> oscillation  (YELLOW)
    4. LEDs + reset button (manual restart after a fault)

  Hardware assumptions (see the project schematic):
    - Latch Q = HIGH means "fault": gates held off, ZVS stopped
    - Latch S is driven by the LM393 and by D6, each through a 1N5819 (diode OR),
      with 100k from S to GND
    - Threshold filter: D3 -> 10k -> vth -> LM393 IN-,  vth -> 15k -> GND,  vth -> 1uF -> GND
    - Shunt 5 mOhm, INA181A1 (gain 20)

  Pin map (Arduino number / port / DIP-28 pin):
    D2  PD2 pin 4   reset button to GND (internal pull-up)
    D3  PD3 pin 5   threshold PWM (Timer2, OC2B)
    D5  PD5 pin 11  osc input (Timer1 external clock T1)
    D6  PD6 pin 12  latch S (via diode)
    D7  PD7 pin 13  latch R
    D8  PB0 pin 14  latch Q (input)
    A0  PC0 pin 23  green LED
    A1  PC1 pin 24  red LED
    A2  PC2 pin 25  yellow LED
    A3  PC3 pin 26  INA181 output (optional, debug only)
    D0/D1           serial debug (115200 baud)
*/

#include <avr/wdt.h>

// ------------------------------------------------------------------
// Pins
// ------------------------------------------------------------------
const uint8_t PIN_BUTTON  = 2;
const uint8_t PIN_VTH_PWM = 3;
const uint8_t PIN_OSC     = 5;   // documented only: Timer1 reads T1 directly
const uint8_t PIN_LATCH_S = 6;
const uint8_t PIN_LATCH_R = 7;
const uint8_t PIN_LATCH_Q = 8;
const uint8_t PIN_LED_GRN = A0;
const uint8_t PIN_LED_RED = A1;
const uint8_t PIN_LED_YEL = A2;
const uint8_t PIN_ISENSE  = A3;

// ------------------------------------------------------------------
// Current sensing and threshold settings
// ------------------------------------------------------------------
const float R_SHUNT      = 0.005;       // ohm
const float SENSE_GAIN   = 20.0;        // INA181A1
const float VCC_VOLTS    = 5.0;
const float DIVIDER      = 15.0 / 25.0; // 10k / 15k divider after the PWM pin

const float I_TRIP_START = 30.0;        // A, hard limit during startup
const float I_TRIP_RUN   = 8.0;         // A, normal limit

// ------------------------------------------------------------------
// Timing and frequency window
// ------------------------------------------------------------------
const uint32_t FILTER_SETTLE_MS    = 50;      // let the 1 uF threshold filter settle
const uint32_t BLANK_MS            = 30;      // keep the hard limit at least this long
const uint32_t STARTUP_TIMEOUT_MS  = 200;     // oscillation must be valid by then
const uint32_t WINDOW_US           = 10000;   // frequency measurement window (10 ms)
const uint32_t FREQ_MIN_HZ         = 90000;   // adjust after the LTspice k-sweep
const uint32_t FREQ_MAX_HZ         = 130000;
const uint8_t  GOOD_WINDOWS_TO_RUN = 3;       // consecutive valid windows before RUN
const uint8_t  BAD_WINDOWS_TO_TRIP = 2;       // consecutive invalid windows before trip
const uint32_t DEBOUNCE_MS         = 30;

const bool     DEBUG               = true;    // serial messages
const uint32_t DEBUG_PERIOD_MS     = 500;

// ------------------------------------------------------------------
// State machine
// ------------------------------------------------------------------
enum State { BOOT, STARTUP, RUN, FAULT_OC, FAULT_OSC };

State    state;
uint32_t stateStartMs = 0;
uint8_t  goodWindows  = 0;
uint8_t  badWindows   = 0;
uint32_t lastDebugMs  = 0;
uint32_t lastFreqHz   = 0;

// Frequency measurement
uint16_t lastCount    = 0;
uint32_t lastWindowUs = 0;

// ------------------------------------------------------------------
// Helpers
// ------------------------------------------------------------------
const char* stateName(State s) {
  switch (s) {
    case BOOT:      return "BOOT";
    case STARTUP:   return "STARTUP";
    case RUN:       return "RUN";
    case FAULT_OC:  return "FAULT_OVERCURRENT";
    case FAULT_OSC: return "FAULT_OSCILLATION";
  }
  return "?";
}

void enterState(State s) {
  state = s;
  stateStartMs = millis();
  if (DEBUG) {
    Serial.print(F("-> "));
    Serial.println(stateName(s));
  }
}

// PWM value (0..255) that gives the threshold voltage for a given trip current
uint8_t pwmForCurrent(float amps) {
  float vth  = amps * R_SHUNT * SENSE_GAIN;
  float duty = vth / (VCC_VOLTS * DIVIDER);
  if (duty > 1.0) duty = 1.0;
  if (duty < 0.0) duty = 0.0;
  return (uint8_t)(duty * 255.0 + 0.5);
}

void setThreshold(float amps) {
  analogWrite(PIN_VTH_PWM, pwmForCurrent(amps));
}

void setLeds(bool g, bool r, bool y) {
  digitalWrite(PIN_LED_GRN, g);
  digitalWrite(PIN_LED_RED, r);
  digitalWrite(PIN_LED_YEL, y);
}

// Short pulse on S: the latch holds the fault by itself afterwards,
// so S is never left HIGH (S and R must never be HIGH together)
void latchSet() {
  digitalWrite(PIN_LATCH_S, HIGH);
  delayMicroseconds(20);
  digitalWrite(PIN_LATCH_S, LOW);
}

void latchReset() {
  digitalWrite(PIN_LATCH_S, LOW);
  digitalWrite(PIN_LATCH_R, HIGH);
  delayMicroseconds(20);
  digitalWrite(PIN_LATCH_R, LOW);
}

bool latchIsSet() {
  return digitalRead(PIN_LATCH_Q) == HIGH;
}

void setupTimers() {
  // Timer1: count external pulses on T1 (D5), rising edge, normal mode
  TCCR1A = 0;
  TCCR1B = _BV(CS12) | _BV(CS11) | _BV(CS10);
  TCNT1  = 0;

  // Timer2: prescaler 1 -> ~31.4 kHz PWM on D3 (default is only ~490 Hz)
  TCCR2B = (TCCR2B & 0b11111000) | 0b001;
}

void resetFreqWindow() {
  lastCount    = TCNT1;
  lastWindowUs = micros();
}

// Returns true once per window, with the measured frequency in Hz
bool measureFreq(uint32_t &freqHz) {
  uint32_t now = micros();
  uint32_t dt  = now - lastWindowUs;
  if (dt < WINDOW_US) return false;

  uint16_t count  = TCNT1;
  uint16_t pulses = count - lastCount;   // wraps correctly
  lastCount    = count;
  lastWindowUs = now;

  freqHz = (uint32_t)pulses * 1000000UL / dt;
  lastFreqHz = freqHz;
  return true;
}

bool freqInWindow(uint32_t f) {
  return f >= FREQ_MIN_HZ && f <= FREQ_MAX_HZ;
}

// Debounced press detection (returns true once per press)
bool buttonPressed() {
  static bool     stable  = HIGH;
  static bool     lastRaw = HIGH;
  static uint32_t tChange = 0;

  bool raw = digitalRead(PIN_BUTTON);
  if (raw != lastRaw) {
    lastRaw = raw;
    tChange = millis();
  }
  if (millis() - tChange >= DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) return true;
  }
  return false;
}

float readCurrentAmps() {
  float v = analogRead(PIN_ISENSE) * VCC_VOLTS / 1023.0;
  return v / (R_SHUNT * SENSE_GAIN);
}

// Hard limit, power stage held off, then BOOT waits for the filter to settle
void startSequence() {
  setThreshold(I_TRIP_START);
  latchSet();
  setLeds(false, false, false);
  enterState(BOOT);
}

// ------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------
void setup() {
  // After a watchdog reset the watchdog stays enabled: switch it off first
  MCUSR = 0;
  wdt_disable();

  pinMode(PIN_LATCH_S, OUTPUT);
  digitalWrite(PIN_LATCH_S, LOW);
  pinMode(PIN_LATCH_R, OUTPUT);
  digitalWrite(PIN_LATCH_R, LOW);
  pinMode(PIN_LATCH_Q, INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_OSC, INPUT);
  pinMode(PIN_VTH_PWM, OUTPUT);
  pinMode(PIN_LED_GRN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YEL, OUTPUT);

  // Unused pins: inputs with pull-up so they do not float
  const uint8_t unused[] = { 4, 9, 10, A4, A5 };
  for (uint8_t i = 0; i < sizeof(unused); i++) pinMode(unused[i], INPUT_PULLUP);

  setupTimers();

  if (DEBUG) {
    Serial.begin(115200);
    Serial.println(F("ZVS supervisor start"));
    Serial.print(F("PWM start = "));
    Serial.print(pwmForCurrent(I_TRIP_START));
    Serial.print(F(", PWM run = "));
    Serial.println(pwmForCurrent(I_TRIP_RUN));
  }

  startSequence();
  wdt_enable(WDTO_250MS);
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop() {
  wdt_reset();

  uint32_t f;
  bool pressed = buttonPressed();

  switch (state) {

    case BOOT:
      setLeds(false, false, false);
      if (millis() - stateStartMs >= FILTER_SETTLE_MS) {
        latchReset();                 // releases the gates: the ZVS starts
        delayMicroseconds(50);
        if (latchIsSet()) {           // set again at once: a fault is still present
          enterState(FAULT_OC);
          break;
        }
        goodWindows = 0;
        badWindows  = 0;
        resetFreqWindow();
        enterState(STARTUP);
      }
      break;

    case STARTUP:
      setLeds((millis() / 100) % 2, false, false);   // green blinking

      if (latchIsSet()) {             // LM393 tripped at the hard limit
        enterState(FAULT_OC);
        break;
      }
      if (measureFreq(f)) {
        goodWindows = freqInWindow(f) ? goodWindows + 1 : 0;
      }
      if (goodWindows >= GOOD_WINDOWS_TO_RUN &&
          millis() - stateStartMs >= BLANK_MS) {
        setThreshold(I_TRIP_RUN);     // blanking over: normal limit
        badWindows = 0;
        enterState(RUN);
      } else if (millis() - stateStartMs >= STARTUP_TIMEOUT_MS) {
        latchSet();                   // never reached a valid frequency
        enterState(FAULT_OSC);
      }
      break;

    case RUN:
      setLeds(true, false, false);

      if (latchIsSet()) {             // checked first: overcurrent stops the
        enterState(FAULT_OC);         // oscillation too, so it must win
        break;
      }
      if (measureFreq(f)) {
        if (freqInWindow(f)) {
          badWindows = 0;
        } else if (++badWindows >= BAD_WINDOWS_TO_TRIP) {
          if (latchIsSet()) {
            enterState(FAULT_OC);
          } else {
            latchSet();
            enterState(FAULT_OSC);
          }
          break;
        }
      }
      if (DEBUG && millis() - lastDebugMs >= DEBUG_PERIOD_MS) {
        lastDebugMs = millis();
        Serial.print(F("f = "));
        Serial.print(lastFreqHz);
        Serial.print(F(" Hz, I = "));
        Serial.print(readCurrentAmps(), 2);
        Serial.println(F(" A"));
      }
      break;

    case FAULT_OC:
      setLeds(false, true, false);
      if (pressed) startSequence();
      break;

    case FAULT_OSC:
      setLeds(false, false, true);
      if (pressed) startSequence();
      break;
  }
}
