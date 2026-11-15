// ASTRAEUS Rev A - flight computer pin map (ESP32-S3-WROOM-1-N8)
// Matches the routed KiCad board. Include this in every ASTRAEUS sketch.
#pragma once

// ---- sensor SPI bus (FSPI): LSM6DSO32 IMU, ADXL375 high-g, BMP388 baro
#define PIN_SPI_SCK     12
#define PIN_SPI_MOSI    11
#define PIN_SPI_MISO    13
#define PIN_CS_IMU      10   // LSM6DSO32  (±32 g, ±2000 dps)
#define PIN_CS_HG       14   // ADXL375    (±200 g)
#define PIN_CS_BARO     18   // BMP388
#define PIN_INT_IMU      8   // LSM6DSO32 INT1
#define PIN_INT_HG       9   // ADXL375 INT1
#define PIN_INT_BARO    17   // BMP388 INT

// ---- microSD, own SPI bus (HSPI) so logging never blocks sensor reads
#define PIN_SD_SCK      36
#define PIN_SD_MOSI     35
#define PIN_SD_MISO     37
#define PIN_SD_CS       38

// ---- pyro channels: low-side AO3400A, 100k gate pull-down (off at boot)
#define PIN_PY1_GATE    40
#define PIN_PY2_GATE    41
#define PIN_PY1_SENSE    1   // ADC: drain voltage x 33/133  -> continuity
#define PIN_PY2_SENSE    2

// ---- servos (TVC): 5 V rail from AP63205, 100R series on each signal
#define PIN_SERVO1       6
#define PIN_SERVO2       7
#define PIN_SERVO3      15
#define PIN_SERVO4      16

// ---- status
#define PIN_BUZZER      21   // passive MLT-8530 via AO3400A, drive 2.7 kHz
#define PIN_LED_STAT    42   // white status LED (high = on)
#define PIN_BOOT         0   // BOOT button, active low (also usable as user button)

// ---- power monitoring (ADC, 11 dB attenuation)
#define PIN_VBAT_SENSE   4   // battery after power switch, x 22/122
#define PIN_ARM_SENSE    5   // pyro rail after ARM switch, x 22/122
#define VBAT_DIVIDER     (122.0f / 22.0f)
#define PYRO_DIVIDER     (133.0f / 33.0f)

// ---- spare GPIO on bottom test pads
#define PIN_SPARE1      47
#define PIN_SPARE2      48
// UART0 TX/RX are also on bottom test pads (GPIO43 / GPIO44)

// ---- buzzer
#define BUZZER_HZ       2700
