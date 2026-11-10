/*
  Emergency Electric Generator (EEG) - Arduino UNO block
  Board: Arduino UNO

  Diagram arrows handled here:
    SENSOR INPUT  -> MCU   D2  (digital sensor, LOW = triggered)
    ANALOG INPUT  -> MCU   A0  (battery voltage through 4:1 divider), A1 (aux, 0-5V direct)
    DIGITAL       <-> MCU  D3  (button, LOW = pressed), D12 (status LED / alarm)
    COMMUNICATE   <-> MCU  USB Serial 9600 baud (text commands)
    MCU -> RELAY CONTROLLER 8 UNITS: D4..D11 = relay 1..8
        relay 5 -> SW -> LOAD,  relay 8 -> BATTERY CHARGER
    POWER RESOURCE 12V -> MCU: feed 12V to VIN (not the 5V pin). No code needed.

  Serial commands (9600 baud, end with Enter / newline):
    STATUS        print readings and relay states
    R<n> <0|1>    set relay n (1-8), e.g.  R5 1   (R8 switches charger to manual)
    ALL 0         all relays OFF (charger switches to manual)
    AUTO <0|1>    1 = battery auto-charge control, 0 = manual
    OUT <0|1>     D12 output
    RESET         clear the sensor fault (only when the sensor is released)
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

// ======================= PIN MAP (edit to match your wiring) =======================
const uint8_t PIN_SENSOR  = 2;    // SENSOR INPUT
const uint8_t PIN_DIG_IN  = 3;    // DIGITAL in (button)
const uint8_t PIN_DIG_OUT = 12;   // DIGITAL out (LED)
const uint8_t PIN_BAT     = A0;   // ANALOG INPUT 1 (battery via divider)
const uint8_t PIN_AUX     = A1;   // ANALOG INPUT 2 (0-5V only)

// Relay number 1..8 -> Arduino pin. Index 0 is unused. Do not use D0/D1 (Serial).
const uint8_t RELAY_PIN[9] = {0, 4, 5, 6, 7, 8, 9, 10, 11};
const uint8_t RELAY_LOAD    = 5;  // relay NUMBER (1-8) to SW -> LOAD
const uint8_t RELAY_CHARGER = 8;  // relay NUMBER (1-8) to BATTERY CHARGER

// ======================= SETTINGS =======================
const bool  RELAY_ACTIVE_LOW = true;   // most relay modules: LOW = ON. false if yours is HIGH = ON
const uint8_t SENSOR_ACTIVE  = LOW;    // level that means "sensor triggered"
const uint8_t BTN_ACTIVE     = LOW;    // level that means "button pressed"
const bool  SENSOR_LATCH     = true;   // true: fault stays until RESET; false: clears when sensor releases

const float ADC_REF      = 5.0;        // Arduino supply voltage (measure your 5V and edit for accuracy)
const float DIVIDER_BAT  = 4.0;        // (100k*3 + 100k) / 100k. Calibrate with a multimeter.
const float DIVIDER_AUX  = 1.0;        // A1 direct, no divider

const float BAT_ON_V        = 12.0;    // charger ON  below this voltage
const float BAT_OFF_V       = 13.2;    // charger OFF at/above this voltage (adjust to battery type)
const float BAT_VALID_MIN_V = 3.0;     // below this = divider/wiring problem, charger forced OFF

const unsigned long BAT_READ_MS = 200;
const unsigned long REPORT_MS   = 1000;
const unsigned long DEBOUNCE_MS = 50;
const uint8_t ADC_SAMPLES       = 16;  // averaging (divider source resistance is high)

// ======================= STATE =======================
bool  relayOn[9]   = {false};
bool  autoCharger  = true;
bool  sensorFault  = false;
float batV         = 0.0;
unsigned long lastBatRead = 0;
unsigned long lastReport  = 0;

const uint8_t RX_MAX = 24;
char    rxBuf[RX_MAX];
uint8_t rxLen = 0;
bool    rxOverflow = false;

// ======================= HELPERS =======================
// n = relay number 1..8
void setRelay(uint8_t n, bool on) {
  if (n < 1 || n > 8) return;
  relayOn[n] = on;
  digitalWrite(RELAY_PIN[n], (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

float readVolt(uint8_t pin, float divider) {
  analogRead(pin);                       // throw away first sample after mux switch
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += analogRead(pin);
  return ((float)sum / ADC_SAMPLES) * (ADC_REF / 1023.0) * divider;
}

// Load relay (relay 5): refuse to turn ON while the sensor fault is active
bool requestLoad(bool on) {
  if (on && sensorFault) {
    Serial.println(F("DENIED: fault active (release sensor, then send RESET)"));
    return false;
  }
  setRelay(RELAY_LOAD, on);
  return true;
}

void printStatus() {
  Serial.print(F("BAT="));    Serial.print(batV, 2);
  Serial.print(F("V AUX="));  Serial.print(readVolt(PIN_AUX, DIVIDER_AUX), 2);
  Serial.print(F("V SENSOR=")); Serial.print(digitalRead(PIN_SENSOR));
  Serial.print(F(" DIG_IN="));  Serial.print(digitalRead(PIN_DIG_IN));
  Serial.print(F(" RELAYS="));
  for (uint8_t n = 1; n <= 8; n++) Serial.print(relayOn[n] ? '1' : '0');
  Serial.print(F(" AUTO="));   Serial.print(autoCharger ? 1 : 0);
  Serial.print(F(" FAULT="));  Serial.println(sensorFault ? 1 : 0);
}

// ======================= COMMUNICATE =======================
void handleCommand(char *s) {
  for (char *p = s; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;   // to upper case
  int n, v;

  if (strcmp(s, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(s, "RESET") == 0) {
    if (digitalRead(PIN_SENSOR) == SENSOR_ACTIVE) {
      Serial.println(F("DENIED: sensor still active"));
    } else {
      sensorFault = false;
      digitalWrite(PIN_DIG_OUT, LOW);
      Serial.println(F("OK"));
    }
  } else if (sscanf(s, "R%d %d", &n, &v) == 2 && n >= 1 && n <= 8 && (v == 0 || v == 1)) {
    if (n == RELAY_CHARGER) autoCharger = false;        // manual command overrides auto control
    if (n == RELAY_LOAD) { if (requestLoad(v == 1)) Serial.println(F("OK")); }
    else { setRelay((uint8_t)n, v == 1); Serial.println(F("OK")); }
  } else if (sscanf(s, "ALL %d", &v) == 1 && v == 0) {
    for (uint8_t i = 1; i <= 8; i++) setRelay(i, false);
    autoCharger = false;
    Serial.println(F("OK"));
  } else if (sscanf(s, "AUTO %d", &v) == 1 && (v == 0 || v == 1)) {
    autoCharger = (v == 1);
    Serial.println(F("OK"));
  } else if (sscanf(s, "OUT %d", &v) == 1 && (v == 0 || v == 1)) {
    digitalWrite(PIN_DIG_OUT, v ? HIGH : LOW);
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERR"));
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxOverflow) { Serial.println(F("ERR")); rxOverflow = false; rxLen = 0; }
      else if (rxLen > 0) { rxBuf[rxLen] = 0; handleCommand(rxBuf); rxLen = 0; }
    } else if (rxLen < RX_MAX - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxOverflow = true;
    }
  }
}

// ======================= SENSOR INPUT =======================
void handleSensor() {
  bool trig = (digitalRead(PIN_SENSOR) == SENSOR_ACTIVE);
  if (trig && !sensorFault) {
    sensorFault = true;
    setRelay(RELAY_LOAD, false);
    digitalWrite(PIN_DIG_OUT, HIGH);
    Serial.println(F("FAULT: sensor triggered, load OFF"));
  } else if (!SENSOR_LATCH && !trig && sensorFault) {
    sensorFault = false;
    digitalWrite(PIN_DIG_OUT, LOW);
    Serial.println(F("FAULT cleared"));
  }
}

// ======================= DIGITAL INPUT (button, debounced) =======================
void handleButton() {
  static uint8_t lastRaw = HIGH, stable = HIGH;
  static unsigned long tChange = 0;
  uint8_t raw = digitalRead(PIN_DIG_IN);
  if (raw != lastRaw) { lastRaw = raw; tChange = millis(); }
  if (stable != lastRaw && (millis() - tChange) >= DEBOUNCE_MS) {
    stable = lastRaw;
    if (stable == BTN_ACTIVE) requestLoad(!relayOn[RELAY_LOAD]);   // press = toggle load
  }
}

// ======================= ANALOG INPUT (battery -> relay 8) =======================
void handleBattery() {
  unsigned long now = millis();
  if (now - lastBatRead < BAT_READ_MS) return;
  lastBatRead = now;
  batV = readVolt(PIN_BAT, DIVIDER_BAT);
  if (!autoCharger) return;

  if (batV < BAT_VALID_MIN_V) {
    setRelay(RELAY_CHARGER, false);                // reading invalid -> never charge blindly
  } else if (batV < BAT_ON_V) {
    setRelay(RELAY_CHARGER, true);
  } else if (batV >= BAT_OFF_V) {
    setRelay(RELAY_CHARGER, false);
  }                                                // in between: keep previous state (hysteresis)
}

// ======================= MAIN =======================
void setup() {
  Serial.begin(9600);

  for (uint8_t n = 1; n <= 8; n++) {               // set OFF level first, then make it an output
    digitalWrite(RELAY_PIN[n], RELAY_ACTIVE_LOW ? HIGH : LOW);
    pinMode(RELAY_PIN[n], OUTPUT);
    relayOn[n] = false;
  }
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  pinMode(PIN_DIG_IN, INPUT_PULLUP);
  digitalWrite(PIN_DIG_OUT, LOW);
  pinMode(PIN_DIG_OUT, OUTPUT);

  lastBatRead = millis() - BAT_READ_MS;            // read battery on the first loop
  lastReport  = millis();
  Serial.println(F("EEG Arduino ready"));
}

void loop() {
  readSerial();
  handleSensor();
  handleButton();
  handleBattery();

  if (millis() - lastReport >= REPORT_MS) {
    lastReport = millis();
    printStatus();
  }
}
