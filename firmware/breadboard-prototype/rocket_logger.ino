/*
  rocket_logger.ino  —  ESP32 high-rate barometric + IMU flight data logger
  --------------------------------------------------------------------------
  Hardware : NodeMCU ESP-32S v1.1 (any ESP32-WROOM-32 board works)
             BMP388 barometer   on I2C   (CJMCU-388; 0x77 or 0x76, auto-detected)
             MPU-6500 IMU       on I2C   (GY-6500/GY-9250; 0x68 or 0x69, auto-detected)
             microSD breakout   on VSPI  (SDIO-labelled board: CLK->18, D0->19, CMD->23, D3->5)
             1 tactile button   GPIO4 -> GND (internal pull-up)
             status LED         GPIO2 (on-board LED on most DevKits)
  Libraries: none beyond the ESP32 Arduino core (Wire, SPI, SD, FreeRTOS).
  Board pkg: "esp32 by Espressif Systems" (Boards Manager), board "ESP32 Dev Module".
  Folder   : Arduino IDE wants this file inside a folder named  rocket_logger/

  Design
  - Core 1 : sensorTask  — fixed-rate loop (SAMPLE_HZ). Reads the IMU every tick and the
             barometer whenever its data-ready flag is set, runs the flight state machine,
             pushes a Sample into a FreeRTOS queue. Never touches the SD card.
  - Core 0 : loggerTask  — drains the queue, formats CSV into a 4 KB buffer, writes and
             flushes to SD, handles the button and the LED. SD write stalls (up to ~250 ms
             on cheap cards) are absorbed by the queue, so the sensor rate never hiccups.
  - Logged pressure is RAW (on-chip IIR off by default). The light EMA used by the state
    machine is NOT written to the file, so all filtering can be done offline.

  Serial : 115200 baud. LIVE_PRINT_HZ lines show time, state, altitude, pressure, temperature, |a|;
           a "--" status line every 5 s. With no SD card the logger still runs (live values only).

  Button : the ESP32's own BOOT button (GPIO0) or the external button (GPIO4) - either works
           short press -> writes a MARK event row (useful during bench tests)
           hold 2 s    -> closes the file safely; if no file is open, starts a new one
           (never hold BOOT while pressing EN or plugging in: that enters upload mode)
  Typed  : in the Serial Monitor type a letter + Enter:  c = close file   n = new file
           m = mark   l = live lines on/off   ? = help
  LED    : blip every 2 s = idle, no file open      fast 4 Hz blink = recording, on pad
           solid          = in flight                double blip     = landed, file closing
           10 Hz flicker  = init error (open Serial Monitor at 115200 to see which)

  CSV    : one row per tick. Baro columns are empty on ticks with no new baro conversion
           (pandas: pd.read_csv(f, comment='#'); baro rows: df.dropna(subset=['press_pa'])).
*/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"

// ------------------------------------------------------------------ user config
#define SAMPLE_HZ          200    // loop + IMU log rate. 1000/SAMPLE_HZ must be a whole ms: 100, 200, 250, 500, 1000
#define BARO_ODR_REG       0x01   // BMP388 ODR:  0x00 = 200 Hz, 0x01 = 100 Hz, 0x02 = 50 Hz, 0x03 = 25 Hz
#define BARO_OSR_REG       0x01   // BMP388 OSR:  0x00 = press x1 / temp x1 (required for 200 Hz)
                                  //              0x01 = press x2 / temp x1 (max 100 Hz)  <- default
                                  //              0x03 = press x8 / temp x1 (max 50 Hz, lowest noise)
#define BARO_IIR_REG       0x00   // BMP388 on-chip IIR: 0x00 off (log raw), 0x02 coef 1, 0x04 coef 3, 0x06 coef 7, 0x08 coef 15
#define SD_SPI_MHZ         10     // 10 MHz is safe on a breadboard; 20 once soldered with short wires
#define LOG_FLUSH_MS       1000   // flush interval = maximum data lost on a power cut
#define QUEUE_DEPTH        1024   // samples buffered while the SD card is busy (~5 s at 200 Hz)
#define LIVE_PRINT_HZ      10     // live time + altitude lines on the Serial Monitor (0 = off). SD logging is unaffected
#define PLOTTER_FORMAT     0      // 1 = print "alt:..,alt_filt:..,acc_g:.." for Tools > Serial Plotter instead of text

#define PAD_SAMPLES        200    // baro samples averaged for the pad reference pressure (2 s at 100 Hz)
#define LAUNCH_ACCEL_G     3.0f   // |a| above this for LAUNCH_TICKS consecutive ticks = launch
#define LAUNCH_TICKS       5
#define LAUNCH_ALT_M       10.0f  // ...or filtered altitude above this = launch (backup / vacuum test)
#define BURNOUT_ACCEL_G    1.0f   // |a| below this for LAUNCH_TICKS consecutive ticks = burnout
#define LAUNCH_LOCKOUT_MS  1500   // no apogee decision this soon after launch
#define APOGEE_DROP_M      3.0f   // filtered altitude must fall this far below the running max...
#define APOGEE_HOLD_N      10     // ...for this many consecutive baro samples
#define LANDED_BAND_M      2.0f   // altitude change per second below this...
#define LANDED_SECONDS     10     // ...for this many seconds = landed
#define MAX_FLIGHT_S       600    // safety timeout after launch: treat as landed
#define POST_LANDING_MS    5000   // keep logging this long after landing, then close the file

#define PIN_SDA   21
#define PIN_SCL   22
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_SD_CS  5
#define PIN_BTN    4
#define PIN_BOOT   0      // the board's own BOOT button: a second control button, no wiring needed
#define PIN_LED    2

// ------------------------------------------------------------------ types / shared state
enum State : uint8_t { ST_PAD = 0, ST_BOOST, ST_COAST, ST_DESCENT, ST_LANDED };
enum Event : uint8_t { EV_NONE = 0, EV_P0_SET, EV_LAUNCH, EV_BURNOUT, EV_APOGEE, EV_LANDED,
                       EV_TIMEOUT, EV_MARK, EV_LOG_START, EV_LOG_STOP };
static const char* EV_NAME[] = { "", "P0_SET", "LAUNCH", "BURNOUT", "APOGEE", "LANDED",
                                 "TIMEOUT", "MARK", "LOG_START", "LOG_STOP" };
static const char* ST_NAME[] = { "PAD", "BOOST", "COAST", "DESCENT", "LANDED" };

struct Sample {
  int64_t  t_us;                              // microseconds since boot
  int16_t  ax, ay, az, gx, gy, gz, imuT;      // raw IMU counts (±16 g -> 2048 LSB/g, ±2000 dps -> 16.4 LSB/dps)
  float    press;                             // Pa, NAN if no new baro conversion this tick
  float    temp;                              // degC from the BMP388
  float    alt;                               // m above pad, raw (unfiltered), NAN until P0 is set
  uint8_t  state;
  uint8_t  event;
};

static QueueHandle_t     q;
static volatile bool     g_resetReq  = false;    // logger -> sensor: new file, re-zero pad reference
static volatile uint8_t  g_injectEv  = EV_NONE;  // logger -> sensor: event to stamp on the next sample
static volatile float    g_p0        = NAN;
static volatile float    g_altF      = 0;        // filtered altitude (status print only)
static volatile float    g_maxAlt    = 0;
static volatile float    g_lastP     = NAN;      // latest pressure (Pa), temperature (C), raw altitude (m), |a| (g)
static volatile float    g_lastT     = NAN;      //   - for the live Serial print only
static volatile float    g_lastAlt   = NAN;
static volatile float    g_accG      = 0;
static volatile float    g_ax = 0, g_ay = 0, g_az = 0;   // latest acceleration per axis (g), live print only
static bool              g_live      = (LIVE_PRINT_HZ > 0); // live lines on/off (type "l" in the Serial Monitor)
static bool              sdOK        = false;    // SD card found; without it the logger still runs and prints live values
static volatile uint8_t  g_state     = ST_PAD;
static volatile uint32_t g_dropped   = 0, g_imuErr = 0, g_baroErr = 0, g_baroN = 0;

// ------------------------------------------------------------------ I2C helpers
static bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;      // repeated start
  if (Wire.requestFrom((uint8_t)addr, (uint8_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// ------------------------------------------------------------------ MPU-6500 (register-level driver)
static uint8_t mpuAddr = 0x68, mpuWho = 0;

static bool mpuInit() {
  const uint8_t cands[2] = { 0x68, 0x69 };
  for (int i = 0; i < 2; i++) {
    uint8_t who = 0;
    if (i2cRead(cands[i], 0x75, &who, 1)) { mpuAddr = cands[i]; mpuWho = who; break; }
  }
  // 0x70 = MPU-6500, 0x68 = MPU-6050, 0x71 = MPU-9250, 0x73 = MPU-9255 (all share these registers).
  // Many GY-6500/GY-9250 boards carry clone chips that answer with another ID but behave the same,
  // so any answer is accepted; only "no answer" (0x00 / 0xFF) is treated as missing.
  if (mpuWho == 0x00 || mpuWho == 0xFF) return false;
  if (!(mpuWho == 0x70 || mpuWho == 0x68 || mpuWho == 0x71 || mpuWho == 0x73))
    Serial.printf("WARNING: unexpected IMU WHO_AM_I 0x%02X (clone chip?) - continuing\n", mpuWho);
  i2cWrite8(mpuAddr, 0x6B, 0x80); delay(100);      // PWR_MGMT_1: device reset
  i2cWrite8(mpuAddr, 0x6B, 0x01); delay(10);       // clock = PLL, sleep off
  i2cWrite8(mpuAddr, 0x6C, 0x00);                  // PWR_MGMT_2: all axes on
  i2cWrite8(mpuAddr, 0x19, 0x00);                  // SMPLRT_DIV: internal rate 1 kHz
  i2cWrite8(mpuAddr, 0x1A, 0x02);                  // CONFIG: gyro DLPF 92 Hz
  i2cWrite8(mpuAddr, 0x1B, 0x18);                  // GYRO_CONFIG: ±2000 dps
  i2cWrite8(mpuAddr, 0x1C, 0x18);                  // ACCEL_CONFIG: ±16 g
  i2cWrite8(mpuAddr, 0x1D, 0x02);                  // ACCEL_CONFIG2: accel DLPF 92 Hz
  return true;
}

static bool mpuRead(int16_t* a, int16_t* g, int16_t* t) {
  uint8_t b[14];
  if (!i2cRead(mpuAddr, 0x3B, b, 14)) return false;   // ACCEL_XOUT_H .. GYRO_ZOUT_L
  a[0] = (int16_t)((b[0] << 8) | b[1]);
  a[1] = (int16_t)((b[2] << 8) | b[3]);
  a[2] = (int16_t)((b[4] << 8) | b[5]);
  *t   = (int16_t)((b[6] << 8) | b[7]);
  g[0] = (int16_t)((b[8] << 8) | b[9]);
  g[1] = (int16_t)((b[10] << 8) | b[11]);
  g[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

// ------------------------------------------------------------------ BMP388 (register-level driver)
static uint8_t bmpAddr = 0x77, bmpId = 0;
struct BmpCal { double t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11; };
static BmpCal cal;
static double bmpTlin = 0;

static bool bmpInit() {
  const uint8_t cands[2] = { 0x77, 0x76 };
  for (int i = 0; i < 2; i++) {
    uint8_t id = 0;
    if (i2cRead(cands[i], 0x00, &id, 1) && (id == 0x50 || id == 0x60)) { bmpAddr = cands[i]; bmpId = id; break; }
  }
  if (!(bmpId == 0x50 || bmpId == 0x60)) return false;    // 0x50 = BMP388, 0x60 = BMP390

  i2cWrite8(bmpAddr, 0x7E, 0xB6); delay(10);              // soft reset

  uint8_t c[21];                                          // NVM_PAR_T1 .. NVM_PAR_P11 at 0x31..0x45
  if (!i2cRead(bmpAddr, 0x31, c, 21)) return false;
  uint16_t t1  = c[0]  | (c[1]  << 8);
  uint16_t t2  = c[2]  | (c[3]  << 8);
  int8_t   t3  = (int8_t)c[4];
  int16_t  p1  = (int16_t)(c[5]  | (c[6]  << 8));
  int16_t  p2  = (int16_t)(c[7]  | (c[8]  << 8));
  int8_t   p3  = (int8_t)c[9];
  int8_t   p4  = (int8_t)c[10];
  uint16_t p5  = c[11] | (c[12] << 8);
  uint16_t p6  = c[13] | (c[14] << 8);
  int8_t   p7  = (int8_t)c[15];
  int8_t   p8  = (int8_t)c[16];
  int16_t  p9  = (int16_t)(c[17] | (c[18] << 8));
  int8_t   p10 = (int8_t)c[19];
  int8_t   p11 = (int8_t)c[20];
  // Scaling per Bosch datasheet section 9.1 (floating-point variant)
  cal.t1  = t1 / 0.00390625;                    // 2^-8
  cal.t2  = t2 / 1073741824.0;                  // 2^30
  cal.t3  = t3 / 281474976710656.0;             // 2^48
  cal.p1  = (p1 - 16384.0) / 1048576.0;         // 2^20
  cal.p2  = (p2 - 16384.0) / 536870912.0;       // 2^29
  cal.p3  = p3 / 4294967296.0;                  // 2^32
  cal.p4  = p4 / 137438953472.0;                // 2^37
  cal.p5  = p5 / 0.125;                         // 2^-3
  cal.p6  = p6 / 64.0;                          // 2^6
  cal.p7  = p7 / 256.0;                         // 2^8
  cal.p8  = p8 / 32768.0;                       // 2^15
  cal.p9  = p9 / 281474976710656.0;             // 2^48
  cal.p10 = p10 / 281474976710656.0;            // 2^48
  cal.p11 = p11 / 36893488147419103232.0;       // 2^65

  i2cWrite8(bmpAddr, 0x1C, BARO_OSR_REG);       // OSR
  i2cWrite8(bmpAddr, 0x1D, BARO_ODR_REG);       // ODR
  i2cWrite8(bmpAddr, 0x1F, BARO_IIR_REG);       // CONFIG (IIR)
  i2cWrite8(bmpAddr, 0x1B, 0x33);               // PWR_CTRL: press_en | temp_en | normal mode
  delay(50);
  return true;
}

static double bmpCompT(uint32_t ut) {
  double pd1 = (double)ut - cal.t1;
  double pd2 = pd1 * cal.t2;
  bmpTlin = pd2 + pd1 * pd1 * cal.t3;
  return bmpTlin;
}

static double bmpCompP(uint32_t up) {
  double t = bmpTlin, u = (double)up;
  double pd1 = cal.p6 * t;
  double pd2 = cal.p7 * t * t;
  double pd3 = cal.p8 * t * t * t;
  double out1 = cal.p5 + pd1 + pd2 + pd3;
  pd1 = cal.p2 * t;
  pd2 = cal.p3 * t * t;
  pd3 = cal.p4 * t * t * t;
  double out2 = u * (cal.p1 + pd1 + pd2 + pd3);
  pd1 = u * u;
  pd2 = cal.p9 + cal.p10 * t;
  pd3 = pd1 * pd2;
  double pd4 = pd3 + u * u * u * cal.p11;
  return out1 + out2 + pd4;                     // Pa
}

// returns 1 = new sample, 0 = nothing new yet, -1 = I2C error
static int bmpRead(float* pressPa, float* tempC) {
  uint8_t b[7];
  if (!i2cRead(bmpAddr, 0x03, b, 7)) return -1;           // STATUS + DATA_0..DATA_5 in one burst
  if (!(b[0] & 0x20)) return 0;                           // drdy_press not set
  uint32_t up = (uint32_t)b[1] | ((uint32_t)b[2] << 8) | ((uint32_t)b[3] << 16);
  uint32_t ut = (uint32_t)b[4] | ((uint32_t)b[5] << 8) | ((uint32_t)b[6] << 16);
  *tempC   = (float)bmpCompT(ut);                          // temperature first: pressure uses t_lin
  *pressPa = (float)bmpCompP(up);
  return 1;
}

static inline float altFromP(float p, float p0) {
  return 44330.0f * (1.0f - powf(p / p0, 0.1902949f));    // ISA barometric formula, relative to pad
}

// ------------------------------------------------------------------ sensor task (core 1)
static void sensorTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
  TickType_t last = xTaskGetTickCount();

  uint8_t  state = ST_PAD;
  double   padSum = 0; int padN = 0; float p0 = NAN;
  float    altF = 0, maxAlt = 0;
  int      launchTicks = 0, burnoutTicks = 0, apogeeN = 0, stableS = 0;
  int64_t  launchT = 0, lastSecT = 0; float altLastSec = 0;

  for (;;) {
    vTaskDelayUntil(&last, period);

    if (g_resetReq) {                                     // new log file: start over
      g_resetReq = false;
      state = ST_PAD; padSum = 0; padN = 0; p0 = NAN; g_p0 = NAN; g_lastAlt = NAN;
      altF = 0; maxAlt = 0; launchTicks = burnoutTicks = apogeeN = stableS = 0;
    }

    Sample s;
    s.t_us = esp_timer_get_time();
    s.press = s.temp = s.alt = NAN;
    s.event = EV_NONE;

    // ---- IMU: every tick
    int16_t a[3], g[3], t;
    if (mpuRead(a, g, &t)) {
      s.ax = a[0]; s.ay = a[1]; s.az = a[2]; s.gx = g[0]; s.gy = g[1]; s.gz = g[2]; s.imuT = t;
    } else {
      g_imuErr++; s.ax = s.ay = s.az = s.gx = s.gy = s.gz = s.imuT = 0;
    }
    float accG = sqrtf((float)s.ax * s.ax + (float)s.ay * s.ay + (float)s.az * s.az) / 2048.0f;
    g_accG = accG;
    g_ax = s.ax / 2048.0f; g_ay = s.ay / 2048.0f; g_az = s.az / 2048.0f;

    // ---- Baro: only when the chip has finished a new conversion
    float p, tc;
    int r = bmpRead(&p, &tc);
    bool newBaro = (r == 1);
    if (r < 0) g_baroErr++;
    if (newBaro) {
      g_baroN++;
      s.press = p; s.temp = tc;
      g_lastP = p; g_lastT = tc;
      if (isnan(p0)) {                                    // still averaging the pad reference
        padSum += p; padN++;
        if (padN >= PAD_SAMPLES) { p0 = (float)(padSum / padN); g_p0 = p0; s.event = EV_P0_SET; altF = 0; }
      } else {
        s.alt = altFromP(p, p0);                          // raw altitude goes to the file
        altF += 0.2f * (s.alt - altF);                    // light EMA, state machine only
        g_altF = altF; g_lastAlt = s.alt;
      }
    }

    // ---- Flight state machine
    int64_t now = s.t_us;
    switch (state) {
      case ST_PAD:
        launchTicks = (accG > LAUNCH_ACCEL_G) ? launchTicks + 1 : 0;
        if (launchTicks >= LAUNCH_TICKS || (!isnan(p0) && altF > LAUNCH_ALT_M)) {
          state = ST_BOOST; launchT = now; maxAlt = altF; s.event = EV_LAUNCH;
          burnoutTicks = 0; apogeeN = 0;
        }
        break;

      case ST_BOOST:
      case ST_COAST:
        if (newBaro && !isnan(p0)) {
          if (altF > maxAlt) maxAlt = altF;
          if (now - launchT > (int64_t)LAUNCH_LOCKOUT_MS * 1000) {
            apogeeN = (altF < maxAlt - APOGEE_DROP_M) ? apogeeN + 1 : 0;
            if (apogeeN >= APOGEE_HOLD_N) {
              state = ST_DESCENT; s.event = EV_APOGEE;
              lastSecT = now; altLastSec = altF; stableS = 0;
            }
          }
        }
        if (state == ST_BOOST) {
          burnoutTicks = (accG < BURNOUT_ACCEL_G) ? burnoutTicks + 1 : 0;
          if (burnoutTicks >= LAUNCH_TICKS) { state = ST_COAST; if (s.event == EV_NONE) s.event = EV_BURNOUT; }
        }
        break;

      case ST_DESCENT:
        if (now - lastSecT >= 1000000) {                  // once per second
          lastSecT = now;
          stableS = (fabsf(altF - altLastSec) < LANDED_BAND_M) ? stableS + 1 : 0;
          altLastSec = altF;
          if (stableS >= LANDED_SECONDS) { state = ST_LANDED; s.event = EV_LANDED; }
        }
        break;

      case ST_LANDED:
        break;
    }
    if (state != ST_PAD && state != ST_LANDED && now - launchT > (int64_t)MAX_FLIGHT_S * 1000000) {
      state = ST_LANDED; s.event = EV_TIMEOUT;
    }

    if (s.event == EV_NONE && g_injectEv != EV_NONE) { s.event = g_injectEv; g_injectEv = EV_NONE; }
    s.state = state; g_state = state; g_maxAlt = maxAlt;

    if (xQueueSend(q, &s, 0) != pdTRUE) g_dropped++;      // never block the sensor loop
  }
}

// ------------------------------------------------------------------ SD logging (core 0)
static File     logFile;
static bool     logOpen = false;
static char     fileName[24] = "";
static char     wbuf[4096];
static size_t   wused = 0;
static uint32_t bytesWritten = 0;

static void flushBuf(bool sync) {
  if (wused) { logFile.write((const uint8_t*)wbuf, wused); bytesWritten += wused; wused = 0; }
  if (sync) logFile.flush();
}

static bool startNewFile() {
  for (int i = 0; i < 1000; i++) {
    snprintf(fileName, sizeof fileName, "/FLT_%03d.csv", i);
    if (!SD.exists(fileName)) break;
  }
  logFile = SD.open(fileName, FILE_WRITE);
  if (!logFile) return false;
  logFile.printf("# rocket_logger v1  sample_hz=%d  baro_odr_reg=0x%02X  baro_osr_reg=0x%02X  baro_iir_reg=0x%02X  accel_fs=16g  gyro_fs=2000dps\n",
                 SAMPLE_HZ, BARO_ODR_REG, BARO_OSR_REG, BARO_IIR_REG);
  logFile.println("t_us,state,event,press_pa,temp_c,alt_m,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps");
  logFile.printf("%lld,%u,LOG_START,,,,,,,,,\n", (long long)esp_timer_get_time(), (unsigned)g_state);
  logFile.flush();
  wused = 0; bytesWritten = 0; logOpen = true;
  g_resetReq = true;                                      // sensor task re-zeroes the pad reference
  return true;
}

static void closeFile() {
  if (!logOpen) return;
  flushBuf(false);
  logFile.printf("%lld,%u,LOG_STOP,,,,,,,,,\n", (long long)esp_timer_get_time(), (unsigned)g_state);
  logFile.flush();
  logFile.close();
  logOpen = false;
}

static int formatLine(char* out, size_t cap, const Sample& s) {
  char baro[64];
  if (isnan(s.press))    strcpy(baro, ",,");
  else if (isnan(s.alt)) snprintf(baro, sizeof baro, "%.2f,%.2f,", s.press, s.temp);
  else                   snprintf(baro, sizeof baro, "%.2f,%.2f,%.2f", s.press, s.temp, s.alt);
  char ev[32] = "";
  if (s.event == EV_P0_SET) snprintf(ev, sizeof ev, "P0_SET=%.2f", g_p0);
  else if (s.event)         snprintf(ev, sizeof ev, "%s", EV_NAME[s.event]);
  return snprintf(out, cap, "%lld,%u,%s,%s,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                  (long long)s.t_us, (unsigned)s.state, ev, baro,
                  s.ax / 2048.0f, s.ay / 2048.0f, s.az / 2048.0f,
                  s.gx / 16.4f,   s.gy / 16.4f,   s.gz / 16.4f);
}

static void updateLed(uint32_t now) {
  bool on;
  uint8_t st = g_state;
  uint32_t ph = now % 1000;
  if (!logOpen)             on = (now % 2000) < 100;      // blip every 2 s: idle, not recording
  else if (st == ST_PAD)    on = (now % 250)  < 125;      // fast 4 Hz blink: recording on the pad
  else if (st == ST_LANDED) on = ph < 100 || (ph >= 200 && ph < 300);   // double blip: landed
  else                      on = true;                    // solid: in flight
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

// One live line on the Serial Monitor: time since power-on, state, altitude, pressure, temperature, |a|
static void printLive() {
  float alt = g_lastAlt, altF = g_altF, maxA = g_maxAlt, p = g_lastP, t = g_lastT, a = g_accG, p0 = g_p0;
  float ts = esp_timer_get_time() / 1e6f;
  if (PLOTTER_FORMAT) {                                   // Serial Plotter: label:value pairs only
    if (!isnan(alt)) Serial.printf("alt:%.2f,alt_filt:%.2f,acc_g:%.2f\n", alt, altF, a);
    return;
  }
  char altTxt[64];
  if (isnan(p0)) snprintf(altTxt, sizeof altTxt, "alt  zeroing, keep still...          ");
  else           snprintf(altTxt, sizeof altTxt, "alt %7.2f m  filt %7.2f  max %7.2f", alt, altF, maxA);
  Serial.printf("%8.2f s | %-7s | %s | P %9.1f Pa | T %5.1f C | acc %5.2f g (x %5.2f y %5.2f z %5.2f)\n",
                ts, ST_NAME[g_state], altTxt, p, t, a, (float)g_ax, (float)g_ay, (float)g_az);
}

static void printHelp() {
  Serial.println("Commands (type a letter, press Enter):  c = close file   n = new file   m = mark   l = live lines on/off   ? = help\n"
                 "Buttons: BOOT on the board or the external button - short press = mark, hold 2 s = close / new file");
}

// close the file if one is open, otherwise start a new one (hold-2-s action)
static void toggleLogging() {
  if (logOpen) { closeFile(); Serial.printf(">> Closed %s - safe to unplug and remove the card\n", fileName); return; }
  if (!sdOK) sdOK = SD.begin(PIN_SD_CS, SPI, SD_SPI_MHZ * 1000000UL);   // card inserted later? retry
  if (sdOK && startNewFile()) Serial.printf(">> Logging to %s\n", fileName);
  else                        Serial.println(">> Could not open a new file (SD card missing or not FAT32?)");
}

static void loggerTask(void*) {
  char line[160];
  uint32_t lastFlush = millis(), lastStatus = millis(), lastLive = millis(), closeAt = 0, downAt = 0;
  uint32_t samplesThisSec = 0;
  bool wasPressed = false, longFired = false;

  for (;;) {
    Sample s;
    while (xQueueReceive(q, &s, 0) == pdTRUE) {           // drain everything waiting
      samplesThisSec++;
      if (!logOpen) continue;
      int n = formatLine(line, sizeof line, s);
      if (n <= 0 || (size_t)n >= sizeof line) continue;
      if (wused + n > sizeof wbuf) flushBuf(false);
      memcpy(wbuf + wused, line, n); wused += n;
      if (s.event == EV_LANDED || s.event == EV_TIMEOUT) closeAt = millis() + POST_LANDING_MS;
    }

    uint32_t now = millis();
    if (logOpen && now - lastFlush >= LOG_FLUSH_MS) { flushBuf(true); lastFlush = now; }
    if (logOpen && closeAt && (int32_t)(now - closeAt) >= 0) { closeFile(); closeAt = 0; Serial.printf(">> Landed: closed %s\n", fileName); }

    // ---- buttons (BOOT on the board, or the external one on GPIO4): short = MARK, hold 2 s = close / new file
    bool bootDown = (digitalRead(PIN_BOOT) == LOW), extDown = (digitalRead(PIN_BTN) == LOW);
    bool pressed = bootDown || extDown;
    if (pressed && !wasPressed) { downAt = now; longFired = false;
                                  Serial.printf(">> Button PRESSED (%s) - hold 2 s to close/start a file\n", bootDown ? "BOOT" : "external, GPIO4"); }
    if (pressed && !longFired && now - downAt >= 2000) { longFired = true; closeAt = 0; toggleLogging(); }
    if (!pressed && wasPressed && !longFired && now - downAt >= 30) { g_injectEv = EV_MARK; Serial.println(">> MARK written"); }
    wasPressed = pressed;

    // ---- typed commands from the Serial Monitor
    while (Serial.available()) {
      char c = tolower(Serial.read());
      if      (c == 'c') { if (logOpen) { closeAt = 0; toggleLogging(); } else Serial.println(">> No file open"); }
      else if (c == 'n') { if (logOpen) closeFile(); closeAt = 0; toggleLogging(); }
      else if (c == 'm') { g_injectEv = EV_MARK; Serial.println(">> MARK written"); }
      else if (c == 'l') { g_live = !g_live; Serial.printf(">> Live lines %s\n", g_live ? "ON" : "OFF"); }
      else if (c == '?' || c == 'h') printHelp();
    }

    updateLed(now);

    if (LIVE_PRINT_HZ > 0 && g_live && now - lastLive >= 1000 / LIVE_PRINT_HZ) { printLive(); lastLive = now; }

    // status line: every 1 s, or every 5 s when live lines are on; none in Serial Plotter mode
    const uint32_t statusMs = (LIVE_PRINT_HZ > 0 && g_live) ? 5000 : 1000;
    if (now - lastStatus >= statusMs) {
      uint32_t el = now - lastStatus;
      if (!PLOTTER_FORMAT)
        Serial.printf("-- %s | %s | %lu smp/s | baro %lu | queue %u | drop %lu | imuErr %lu baroErr %lu | alt %.1f m (max %.1f) | %lu B\n",
                      logOpen ? fileName : "NOT LOGGING (no SD)", ST_NAME[g_state],
                      (unsigned long)(samplesThisSec * 1000UL / el), (unsigned long)(g_baroN * 1000UL / el),
                      (unsigned)uxQueueMessagesWaiting(q), (unsigned long)g_dropped,
                      (unsigned long)g_imuErr, (unsigned long)g_baroErr, (float)g_altF, (float)g_maxAlt,
                      (unsigned long)bytesWritten);
      samplesThisSec = 0; g_baroN = 0; lastStatus = now;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ------------------------------------------------------------------ setup
static void fail(const char* msg) {
  Serial.printf("INIT FAILED: %s\n", msg);
  for (;;) { digitalWrite(PIN_LED, HIGH); delay(50); digitalWrite(PIN_LED, LOW); delay(50); }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BOOT, INPUT_PULLUP);
  delay(300);
  Serial.println("\nrocket_logger v1.1");
  Serial.printf("External button on GPIO4 reads %s\n", digitalRead(PIN_BTN) == LOW ? "LOW = pressed or shorted to GND (check wiring!)" : "HIGH = released (OK)");
  printHelp();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (!mpuInit()) fail("IMU not found on I2C (check SDA=21, SCL=22, VCC=3V3, GND)");
  Serial.printf("IMU  OK  addr 0x%02X  WHO_AM_I 0x%02X (0x70 = MPU-6500)\n", mpuAddr, mpuWho);
  if (!bmpInit()) fail("BMP388 not found on I2C at 0x77/0x76");
  Serial.printf("BARO OK  addr 0x%02X  chip 0x%02X (0x50 = BMP388)\n", bmpAddr, bmpId);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SD_CS);
  sdOK = SD.begin(PIN_SD_CS, SPI, SD_SPI_MHZ * 1000000UL);
  if (sdOK) Serial.printf("SD   OK  %llu MB\n", (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
  else      Serial.println("SD   NOT FOUND - running WITHOUT logging, live values only.\n"
                           "     (Wire the SD board + insert a FAT32 card, then hold the button 2 s or press EN.)");

  q = xQueueCreate(QUEUE_DEPTH, sizeof(Sample));
  if (!q) fail("queue allocation");
  if (sdOK) {
    if (startNewFile()) Serial.printf("Logging to %s  loop %d Hz, baro at chip ODR, IIR reg 0x%02X\n", fileName, SAMPLE_HZ, BARO_IIR_REG);
    else                Serial.println("Could not create a log file - running WITHOUT logging");
  }
  Serial.println("Zeroing altitude: keep the board still for 2 s...");

  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(loggerTask, "logger", 8192, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));                        // everything runs in the two tasks
}
