/*********************************************************************
 *  ESP32-C3 → Nextion @ 115200: FULL SKETCH
 *  Version: 1.0.2
 *  S1 (D5) → OP1 (D2) → t12
 *  S2 (D6) → OP2 (D3) → t13
 *  S3 (D7) → OP3 (D4) → t14
 *  INA3221 (0x40):
 *    Voltage: CH1→t0,  CH2→t1,  CH3→t2  (xx.xx V)
 *    Current: CH1→t15, CH2→t16, CH3→t17 (xxxx mA)
 *    Power:   CH1→t18, CH2→t19, CH3→t20 (xxxx mW)
 *  MCP23017 (0x27) Fault Indicators (active LOW):
 *    GPA0=TC→t27, GPA1=WAR→t28, GPA2=CRI→t29, GPA3=PV→t30
 *  A0 (CH0) → t3 | A1 (CH1) → t4
 *  LATCHING ON FALLING EDGE (HIGH→LOW)
 *  OP1/OP2/OP3 = LOW AT STARTUP — GUARANTEED
 *  t12/t13/t14 = HMI DEFAULT AT STARTUP (NO CODE CHANGE)
 *  NON-BLOCKING | HARDWARE DEBOUNCED
 *********************************************************************/

#define FW_VERSION "1.0.2"

#include <Wire.h>
#include "INA3221.h"

HardwareSerial Nextion(1);
INA3221 INA(0x40);

// ——— MCP23017 ———
const uint8_t MCP23017_ADDR = 0x27;
const uint8_t MCP_IODIRA    = 0x00;  // Direction register (1=input)
const uint8_t MCP_GPPUA     = 0x0C;  // Pull-up register (1=enabled)
const uint8_t MCP_GPIOA     = 0x12;  // GPIO read register

// ——— OUTPUT POLARITY ———
constexpr bool OUTPUT_ACTIVE_HIGH = true;
// ——————————————————————

// ——— PINS ———
const uint8_t PIN_S1  = 5;  // D5 — Active HIGH from debounce
const uint8_t PIN_S2  = 6;  // D6 — Active HIGH from debounce
const uint8_t PIN_S3  = 7;  // D7 — Active HIGH from debounce
const uint8_t PIN_OP1 = 2;  // D2
const uint8_t PIN_OP2 = 3;  // D3
const uint8_t PIN_OP3 = 4;  // D4

// ——— TIMING ———
const uint32_t VOLTAGE_UPDATE_MS = 50;   // 50 Hz total for INA3221
const uint32_t ADC_UPDATE_MS     = 50;   // 50 Hz total for A0/A1
const uint32_t FAULT_UPDATE_MS   = 100;  // 10 Hz for fault monitoring
uint32_t lastVoltageUpdate = 0;
uint32_t lastADCUpdate     = 0;
uint32_t lastFaultUpdate   = 0;
uint8_t currentChannel     = 0;

// ——— EMA FILTER ———
#define EMA_ALPHA 0.30f
class EMA {
  float value = 0.0f;
  bool first = true;
public:
  float update(float input) {
    if (first) { value = input; first = false; }
    else { value = EMA_ALPHA * input + (1.0f - EMA_ALPHA) * value; }
    return value;
  }
};
EMA busEMA[3];
EMA currentEMA[3];
EMA powerEMA[3];

// ——— NEXTION FIELD MAPPING ———
const uint8_t VOLTAGE_FIELDS[3] = {0, 1, 2};     // t0, t1, t2
const uint8_t CURRENT_FIELDS[3] = {15, 16, 17};  // t15, t16, t17
const uint8_t POWER_FIELDS[3]   = {18, 19, 20};  // t18, t19, t20

// ——— FAULT INDICATORS ———
const uint8_t FAULT_FIELDS[4] = {27, 28, 29, 30};  // t27=TC, t28=WAR, t29=CRI, t30=PV
const uint16_t FAULT_COLOR_OK    = 1728;   // No fault
const uint16_t FAULT_COLOR_FAULT = 63488;  // Fault active
uint8_t lastFaultState = 0xFF;  // Track previous state to avoid unnecessary updates

// ——— SWITCH STATE ———
bool switchState[3] = {false};
bool s1_latched = false;
bool s2_latched = false;
bool s3_latched = false;

// ——— t12/t13/t14 COLORS ———
const uint16_t T_BCO_OFF = 1728;
const uint16_t T_PCO     = 65504;
const uint16_t T_BCO_ON  = 63488;

void setup() {
  Serial.begin(115200);
  Nextion.begin(115200, SERIAL_8N1, 21, 20);

  // ——— 1. FORCE ALL OP LOW FIRST ———
  pinMode(PIN_OP1, OUTPUT);
  pinMode(PIN_OP2, OUTPUT);
  pinMode(PIN_OP3, OUTPUT);
  digitalWrite(PIN_OP1, LOW);
  digitalWrite(PIN_OP2, LOW);
  digitalWrite(PIN_OP3, LOW);

  // ——— 2. Configure inputs ———
  pinMode(PIN_S1, INPUT_PULLUP);
  pinMode(PIN_S2, INPUT_PULLUP);
  pinMode(PIN_S3, INPUT_PULLUP);

  // ——— 3. INA3221 INIT ———
  Wire.begin();
  if (!INA.begin()) {
    Serial.println(F("FATAL: INA3221 not found! Halting."));
    while (true) { delay(1000); }
  }
  Serial.println(F("INA3221 initialized"));
  INA.setShuntR(0, 0.099);
  INA.setShuntR(1, 0.099);
  INA.setShuntR(2, 0.099);
  INA.setAverage(2);

  // ——— 3b. MCP23017 INIT ———
  if (!mcpWriteReg(MCP_IODIRA, 0x0F)) {  // GPA0-3 as inputs, GPA4-7 as outputs
    Serial.println(F("FATAL: MCP23017 not found! Halting."));
    while (true) { delay(1000); }
  }
  mcpWriteReg(MCP_GPPUA, 0x0F);   // Enable pull-ups on GPA0-3
  Serial.println(F("MCP23017 initialized"));

  // ——— 4. ADC CONFIG ———
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // ——— 5. NEXTION INIT ———
  //sendCmd(F("rest"));
  delay(1200);
  sendCmd(F("page 0"));
  delay(400);

  // ——— 6. START ALL SWITCHES OFF ———
  s1_latched = s2_latched = s3_latched = false;
  switchState[0] = switchState[1] = switchState[2] = false;

  lastVoltageUpdate = lastADCUpdate = lastFaultUpdate = millis();

  Serial.println(F("FW v" FW_VERSION " | STARTUP: OP1/OP2/OP3 = LOW | t12/t13/t14 = HMI DEFAULT"));
}

void loop() {
  uint32_t now = millis();

  // ——— 1. UPDATE INA3221 (V/I/P per channel) ———
  if (now - lastVoltageUpdate >= VOLTAGE_UPDATE_MS) {
    lastVoltageUpdate = now;

    // Read all three values for current channel
    float busV = INA.getBusVoltage(currentChannel);
    float mA   = INA.getCurrent_mA(currentChannel);
    float mW   = INA.getPower_mW(currentChannel);

    // Apply EMA filtering
    float filteredV = busEMA[currentChannel].update(busV);
    float filteredI = currentEMA[currentChannel].update(mA);
    float filteredP = powerEMA[currentChannel].update(mW);

    // Format and send voltage (xx.xx V), current (xxxx mA), power (xxxx mW)
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "t%d.txt=\"%05.2f\"", VOLTAGE_FIELDS[currentChannel], filteredV);
    sendCmd(cmd);
    snprintf(cmd, sizeof(cmd), "t%d.txt=\"%04.0f\"", CURRENT_FIELDS[currentChannel], filteredI);
    sendCmd(cmd);
    snprintf(cmd, sizeof(cmd), "t%d.txt=\"%04.0f\"", POWER_FIELDS[currentChannel], filteredP);
    sendCmd(cmd);

    currentChannel = (currentChannel + 1) % 3;
  }

  // ——— 2. UPDATE A0/A1 (t3/t4) ———
  if (now - lastADCUpdate >= ADC_UPDATE_MS) {
    lastADCUpdate = now;

    float v0 = readVoltage(A0);  // A0 = CH0
    float v1 = readVoltage(A1);  // A1 = CH1

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "t3.txt=\"%.2f\"", v0);
    sendCmd(cmd);
    snprintf(cmd, sizeof(cmd), "t4.txt=\"%.2f\"", v1);
    sendCmd(cmd);
  }

  // ——— 3. FAULT MONITORING (MCP23017) ———
  if (now - lastFaultUpdate >= FAULT_UPDATE_MS) {
    lastFaultUpdate = now;
    updateFaultIndicators();
  }

  // ——— 4. SWITCHES & OUTPUTS ———
  updateSwitchesAndOutputs();

  // ——— 5. DEBUG PRINT ———
  static uint32_t lastPrint = 0;
  if (Serial && now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.printf("CH1: %.2fV %.0fmA %.0fmW | CH2: %.2fV %.0fmA %.0fmW | CH3: %.2fV %.0fmA %.0fmW | heap=%lu\n",
                  INA.getBusVoltage(0), INA.getCurrent_mA(0), INA.getPower_mW(0),
                  INA.getBusVoltage(1), INA.getCurrent_mA(1), INA.getPower_mW(1),
                  INA.getBusVoltage(2), INA.getCurrent_mA(2), INA.getPower_mW(2),
                  (unsigned long)ESP.getFreeHeap());
  }
}

void updateSwitchesAndOutputs() {
  // ——— S1 → t12 ———
  static bool s1_last = false;
  bool s1_current = (digitalRead(PIN_S1) == HIGH);
  if (s1_last && !s1_current) {
    s1_latched = !s1_latched;
    switchState[0] = s1_latched;
    digitalWrite(PIN_OP1, s1_latched ? HIGH : LOW);
    updateLabelVisual("t12", s1_latched);
  }
  s1_last = s1_current;

  // ——— S2 → t13 ———
  static bool s2_last = false;
  bool s2_current = (digitalRead(PIN_S2) == HIGH);
  if (s2_last && !s2_current) {
    s2_latched = !s2_latched;
    switchState[1] = s2_latched;
    digitalWrite(PIN_OP2, s2_latched ? HIGH : LOW);
    updateLabelVisual("t13", s2_latched);
  }
  s2_last = s2_current;

  // ——— S3 → t14 ———
  static bool s3_last = false;
  bool s3_current = (digitalRead(PIN_S3) == HIGH);
  if (s3_last && !s3_current) {
    s3_latched = !s3_latched;
    switchState[2] = s3_latched;
    digitalWrite(PIN_OP3, s3_latched ? HIGH : LOW);
    updateLabelVisual("t14", s3_latched);
  }
  s3_last = s3_current;
}

void updateLabelVisual(const char* label, bool is_on) {
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "%s.bco=%u", label, is_on ? T_BCO_ON : T_BCO_OFF);
  sendCmd(cmd);
  snprintf(cmd, sizeof(cmd), "%s.pco=%u", label, T_PCO);
  sendCmd(cmd);
  snprintf(cmd, sizeof(cmd), "%s.ycen=1", label);
  sendCmd(cmd);
  snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", label, is_on ? "ON" : "OFF");
  sendCmd(cmd);
}

float readVoltage(uint8_t channel) {
  int raw = analogRead(channel);
  return raw * 3.3f / 4095.0f;
}

void sendCmd(const char* cmd) {
  Nextion.print(cmd);
  Nextion.write(0xFF); Nextion.write(0xFF); Nextion.write(0xFF);
}
void sendCmd(const __FlashStringHelper* f) {
  Nextion.print(f);
  Nextion.write(0xFF); Nextion.write(0xFF); Nextion.write(0xFF);
}

// ——— MCP23017 FUNCTIONS ———
bool mcpWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("MCP23017 write error: reg=0x%02X err=%d\n", reg, err);
    return false;
  }
  return true;
}

uint8_t mcpReadReg(uint8_t reg, bool* success) {
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.printf("MCP23017 read error: reg=0x%02X err=%d\n", reg, err);
    if (success) *success = false;
    return 0xFF;
  }
  Wire.requestFrom(MCP23017_ADDR, (uint8_t)1);
  if (!Wire.available()) {
    Serial.println(F("MCP23017 no data available"));
    if (success) *success = false;
    return 0xFF;
  }
  if (success) *success = true;
  return Wire.read();
}

void updateFaultIndicators() {
  bool success = false;
  uint8_t gpioa = mcpReadReg(MCP_GPIOA, &success);
  if (!success) return;  // Skip update on I2C error
  uint8_t faultBits = gpioa & 0x0F;  // Only GPA0-3

  // Only update Nextion if state changed
  if (faultBits == lastFaultState) return;
  lastFaultState = faultBits;

  // Check each fault line (active LOW = fault)
  char cmd[20];
  for (uint8_t i = 0; i < 4; i++) {
    bool fault = !(faultBits & (1 << i));  // LOW = fault
    uint16_t color = fault ? FAULT_COLOR_FAULT : FAULT_COLOR_OK;
    snprintf(cmd, sizeof(cmd), "t%d.bco=%u", FAULT_FIELDS[i], color);
    sendCmd(cmd);
  }
}
