/*
  ASTRAEUS Rev A - bring-up self-test  (v0.1)
  -------------------------------------------
  Upload once the board arrives, before any flight firmware.
  Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot: Enabled,
               Flash Size 8MB, PSRAM Disabled. Serial Monitor 115200.

  What it does at power-up:
    1. Rising chirp from the buzzer.
    2. Checks each chip by reading its ID register over SPI:
         LSM6DSO32 WHO_AM_I = 0x6C, ADXL375 DEVID = 0xE5, BMP388 CHIP_ID = 0x50
       and mounts the microSD card.
    3. Reports by sound (so it works inside the rocket):
         one long beep            = everything passed
         N short beeps, repeated  = failure N (1 IMU, 2 high-g, 3 baro, 4 SD)
    4. Then, every 3 s: one beep per pyro channel that has continuity
       (only meaningful with the ARM switch on - that is how commercial
       altimeters report it on the pad).

  Serial commands:
    ?  help         r  read sensors once     s  servo sweep (TVC centre, +-15 deg)
    b  beep test    l  LED test               v  battery and ARM voltages
    FIRE1 / FIRE2   pulse a pyro channel for 0.5 s - BENCH TEST WITH A BULB OR
                    LED + 1k RESISTOR ONLY, never an igniter indoors. Needs the
                    ARM switch on and a 3-second beeping countdown.
*/
#include <SPI.h>
#include <SD.h>
#include "astraeus_pins.h"

SPIClass sensSPI(FSPI);
SPIClass sdSPI(HSPI);
const SPISettings SENS_SPI(5000000, MSBFIRST, SPI_MODE3);   // ADXL375 max 5 MHz

const int SERVO_PINS[4] = {PIN_SERVO1, PIN_SERVO2, PIN_SERVO3, PIN_SERVO4};
bool okIMU, okHG, okBaro, okSD;
int failCode = 0;

// ------------------------------------------------------------------ helpers
uint8_t spiRead(int cs, uint8_t reg, bool dummy = false) {
  sensSPI.beginTransaction(SENS_SPI);
  digitalWrite(cs, LOW);
  sensSPI.transfer(reg | 0x80);
  if (dummy) sensSPI.transfer(0x00);           // BMP388 sends one dummy byte first
  uint8_t v = sensSPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  sensSPI.endTransaction();
  return v;
}
void spiWrite(int cs, uint8_t reg, uint8_t val) {
  sensSPI.beginTransaction(SENS_SPI);
  digitalWrite(cs, LOW);
  sensSPI.transfer(reg & 0x7F);
  sensSPI.transfer(val);
  digitalWrite(cs, HIGH);
  sensSPI.endTransaction();
}
int16_t rd16(int cs, uint8_t reg) {           // little-endian pair, auto-increment
  sensSPI.beginTransaction(SENS_SPI);
  digitalWrite(cs, LOW);
  sensSPI.transfer(reg | 0x80 | (cs == PIN_CS_HG ? 0x40 : 0));
  uint8_t lo = sensSPI.transfer(0), hi = sensSPI.transfer(0);
  digitalWrite(cs, HIGH);
  sensSPI.endTransaction();
  return (int16_t)(hi << 8 | lo);
}

void beep(int ms, int hz = BUZZER_HZ) {
  ledcWriteTone(PIN_BUZZER, hz);
  delay(ms);
  ledcWriteTone(PIN_BUZZER, 0);
}
void beeps(int n, int on = 120, int off = 180) {
  for (int i = 0; i < n; i++) { beep(on); delay(off); }
}
float volts(int pin, float k) { return analogReadMilliVolts(pin) * k / 1000.0f; }

void servoUs(int ch, int us) {                // 50 Hz, 14-bit: 20000 us = 16384 counts
  ledcWrite(SERVO_PINS[ch], (uint32_t)us * 16384 / 20000);
}

// ------------------------------------------------------------------ tests
void checkChips() {
  // BMP388 starts in I2C mode; the first CS edge switches it to SPI
  spiRead(PIN_CS_BARO, 0x00, true);
  uint8_t idImu = spiRead(PIN_CS_IMU, 0x0F);
  uint8_t idHg = spiRead(PIN_CS_HG, 0x00);
  uint8_t idBaro = spiRead(PIN_CS_BARO, 0x00, true);
  okIMU = idImu == 0x6C; okHG = idHg == 0xE5; okBaro = idBaro == 0x50;
  Serial.printf("LSM6DSO32 WHO_AM_I 0x%02X %s\n", idImu, okIMU ? "OK" : "FAIL (expect 0x6C)");
  Serial.printf("ADXL375   DEVID    0x%02X %s\n", idHg, okHG ? "OK" : "FAIL (expect 0xE5)");
  Serial.printf("BMP388    CHIP_ID  0x%02X %s\n", idBaro, okBaro ? "OK" : "FAIL (expect 0x50)");

  if (okIMU) {
    spiWrite(PIN_CS_IMU, 0x12, 0x44);   // CTRL3_C: BDU + auto-increment
    spiWrite(PIN_CS_IMU, 0x10, 0x48);   // CTRL1_XL: 104 Hz, +-8 g (DSO32: FS 10 = 8 g)
    spiWrite(PIN_CS_IMU, 0x11, 0x4C);   // CTRL2_G: 104 Hz, 2000 dps
  }
  if (okHG) {
    spiWrite(PIN_CS_HG, 0x31, 0x0B);    // DATA_FORMAT: 4-wire SPI, full res
    spiWrite(PIN_CS_HG, 0x2C, 0x0A);    // BW_RATE: 100 Hz
    spiWrite(PIN_CS_HG, 0x2D, 0x08);    // POWER_CTL: measure
  }
  if (okBaro) spiWrite(PIN_CS_BARO, 0x1B, 0x33);   // PWR_CTRL: press + temp, normal mode

  okSD = SD.begin(PIN_SD_CS, sdSPI, 20000000);
  if (okSD) Serial.printf("microSD   %llu MB    OK\n", SD.cardSize() / (1024ULL * 1024ULL));
  else      Serial.println("microSD   not found  FAIL (card inserted? FAT32?)");

  failCode = !okIMU ? 1 : !okHG ? 2 : !okBaro ? 3 : !okSD ? 4 : 0;
}

void readSensors() {
  if (okIMU) {
    float k = 8.0f / 32768.0f;
    Serial.printf("IMU   ax %+.2f ay %+.2f az %+.2f g   gx %+6.1f gy %+6.1f gz %+6.1f dps\n",
                  rd16(PIN_CS_IMU, 0x28) * k, rd16(PIN_CS_IMU, 0x2A) * k, rd16(PIN_CS_IMU, 0x2C) * k,
                  rd16(PIN_CS_IMU, 0x22) * 0.07f, rd16(PIN_CS_IMU, 0x24) * 0.07f, rd16(PIN_CS_IMU, 0x26) * 0.07f);
  }
  if (okHG) {
    float k = 0.049f;                     // 49 mg/LSB
    Serial.printf("HIGH-G ax %+.2f ay %+.2f az %+.2f g  (coarse: 1 g is only ~20 counts)\n",
                  rd16(PIN_CS_HG, 0x32) * k, rd16(PIN_CS_HG, 0x34) * k, rd16(PIN_CS_HG, 0x36) * k);
  }
  if (okBaro) {
    uint8_t s = spiRead(PIN_CS_BARO, 0x03, true);
    Serial.printf("BARO  status 0x%02X (raw data ready = %s; full compensation is in the flight code)\n",
                  s, (s & 0x60) == 0x60 ? "yes" : "no");
  }
  Serial.printf("POWER battery %.2f V   pyro rail %.2f V (%s)\n", volts(PIN_VBAT_SENSE, VBAT_DIVIDER),
                volts(PIN_ARM_SENSE, VBAT_DIVIDER), armed() ? "ARMED" : "safe");
  Serial.printf("PYRO  ch1 %.2f V  ch2 %.2f V  (continuity reads close to the pyro rail)\n",
                volts(PIN_PY1_SENSE, PYRO_DIVIDER), volts(PIN_PY2_SENSE, PYRO_DIVIDER));
}

bool armed() { return volts(PIN_ARM_SENSE, VBAT_DIVIDER) > 5.0f; }
bool continuity(int ch) {
  float vr = volts(PIN_ARM_SENSE, VBAT_DIVIDER);
  float vd = volts(ch == 1 ? PIN_PY1_SENSE : PIN_PY2_SENSE, PYRO_DIVIDER);
  return armed() && vd > 0.6f * vr;
}

void servoSweep() {
  Serial.println("Servo sweep: centre, -15, +15, centre on all 4 channels");
  const int seq[] = {1500, 1333, 1667, 1500};
  for (int us : seq) { for (int c = 0; c < 4; c++) servoUs(c, us); delay(600); }
}

void fire(int ch) {
  if (!armed()) { Serial.println("Refused: ARM switch is off."); beeps(3, 60, 60); return; }
  Serial.printf("PYRO %d: 3-second countdown (send any key to abort)\n", ch);
  for (int i = 3; i > 0; i--) {
    beep(150, 3200);
    unsigned long t = millis();
    while (millis() - t < 850) if (Serial.available()) { Serial.println("Aborted."); while (Serial.available()) Serial.read(); return; }
  }
  int pin = ch == 1 ? PIN_PY1_GATE : PIN_PY2_GATE;
  digitalWrite(pin, HIGH); delay(500); digitalWrite(pin, LOW);
  Serial.println("Pulse done.");
}

void help() {
  Serial.println("\nASTRAEUS self-test  |  ? help  r read  s servos  b beep  l LED  v volts  FIRE1/FIRE2 bench pyro test");
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  // pyro gates low first, before anything else
  pinMode(PIN_PY1_GATE, OUTPUT); digitalWrite(PIN_PY1_GATE, LOW);
  pinMode(PIN_PY2_GATE, OUTPUT); digitalWrite(PIN_PY2_GATE, LOW);
  pinMode(PIN_LED_STAT, OUTPUT);
  for (int cs : {PIN_CS_IMU, PIN_CS_HG, PIN_CS_BARO}) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); }
  pinMode(PIN_BOOT, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);
  ledcAttach(PIN_BUZZER, BUZZER_HZ, 8);
  for (int c = 0; c < 4; c++) { ledcAttach(SERVO_PINS[c], 50, 14); servoUs(c, 1500); }

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) delay(10);

  for (int hz = 2000; hz <= 3400; hz += 350) beep(70, hz);   // power-up chirp
  sensSPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  sdSPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  delay(50);

  Serial.println("\n=== ASTRAEUS Rev A self-test ===");
  checkChips();
  if (failCode == 0) { Serial.println("ALL CHECKS PASSED"); beep(900); }
  else Serial.printf("FAILURE CODE %d\n", failCode);
  readSensors();
  help();
}

String cmd;
unsigned long lastReport = 0;

void loop() {
  // status LED: slow blink when healthy, fast when a check failed
  digitalWrite(PIN_LED_STAT, (millis() / (failCode ? 100 : 1000)) % 2);

  if (millis() - lastReport > 3000) {
    lastReport = millis();
    if (failCode) { beeps(failCode); }
    else {
      int n = 0;
      for (int ch = 1; ch <= 2; ch++) if (continuity(ch)) { beep(250, 3000); delay(250); n++; }
      (void)n;
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cmd.trim();
      if (cmd == "?") help();
      else if (cmd == "r") readSensors();
      else if (cmd == "s") servoSweep();
      else if (cmd == "b") { beeps(3); beep(600, 2000); }
      else if (cmd == "l") { for (int i = 0; i < 10; i++) { digitalWrite(PIN_LED_STAT, i % 2); delay(150); } }
      else if (cmd == "v") Serial.printf("battery %.2f V, pyro rail %.2f V\n", volts(PIN_VBAT_SENSE, VBAT_DIVIDER), volts(PIN_ARM_SENSE, VBAT_DIVIDER));
      else if (cmd == "FIRE1") fire(1);
      else if (cmd == "FIRE2") fire(2);
      else if (cmd.length()) Serial.println("Unknown command - type ? for help");
      cmd = "";
    } else cmd += c;
  }

  if (digitalRead(PIN_BOOT) == LOW) {            // BOOT button = re-run the checks
    delay(30);
    while (digitalRead(PIN_BOOT) == LOW) delay(10);
    checkChips();
    if (failCode == 0) beep(900);
  }
}
