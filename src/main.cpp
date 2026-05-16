#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#if __has_include(<esp_wpa2.h>)
#include <esp_wpa2.h>
#define POTENV2_HAS_WPA2_ENTERPRISE 1
#else
#define POTENV2_HAS_WPA2_ENTERPRISE 0
#endif
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "esp_timer.h"

// ======================================================
// smart-ec Multitasking Example
// ESP32-S3 + AD5680 + AD7694 + LMP91000 + TFT ILI9488
//
// Core 1 : measurementTask, dynamic priority
// Core 0 : optionTask, low priority for Serial / logging / I2C / manual TFT refresh
//
// Important scheduler note:
// measurementTask calls vTaskDelay(1) after each point so optionTask can print logs
// and receive Serial commands. taskYIELD() is not enough for a lower-priority task.
//
// Important:
// DAC, ADC, TFT, Touch share the same physical SPI pins.
// TFT uses LovyanGFX Hardware SPI.
// DAC/ADC use the same Hardware SPI bus and every SPI user must use spiMutex.
// ======================================================

// ======================================================
// Compile options
// ======================================================
#define ENABLE_TFT_DISPLAY       1
#define ENABLE_TOUCH_CONTROL     1
#define ENABLE_TOUCH_POLLING     1
#define ENABLE_I2C_MONITOR_TASK  0

// ======================================================
// Pin Mapping
// ======================================================

// I2C LMP91000
#define I2C_SDA 13
#define I2C_SCL 14

// SPI shared bus
#define SPI_SCK   5
#define SPI_MOSI 16   // AD5680 DIN / TFT MOSI / Touch MOSI
#define SPI_MISO 17   // AD7694 SDO / TFT MISO / Touch MISO

// DAC AD5680
#define DAC_SYNC 11

// ADC AD7694
// Important: CNV is kept HIGH when idle so AD7694 releases shared MISO.
#define ADC_CNV 12

// TFT MSP3520 / ILI9488 pins
// ใช้ค่าตามไฟล์ TestMeasurementSystems4_LovyanGFX_SharedSPI.cpp
// ถ้าบอร์ดจริงใช้ CS/DC คนละขา ให้แก้ตรงนี้จุดเดียว
#define TFT_CS   10
#define TFT_DC   21
#define TFT_RST  4
#define TFT_BL   -1

// Touch XPT2046 pins
// Touch shares the same SPI bus as TFT/DAC/ADC, but must have its own CS.
#define TOUCH_CS   7
#define TOUCH_CLK  SPI_SCK
#define TOUCH_MOSI SPI_MOSI
#define TOUCH_MISO SPI_MISO
#define TOUCH_IRQ  15

// Touch calibration storage
#define CALIBRATION_FILE "/TouchCalData"

// LMP91000 MENB
// If MENB is not connected, use -1.
#define LMP_MENB_PIN -1

// Optional built-in LED
#define LED_BUILTIN_PIN 2

// ======================================================
// Voltage reference
// ======================================================
static const float DAC_VREF = 4.096f;
static const float ADC_VREF = 4.096f;
static const uint32_t DAC_MAX_CODE = 0x3FFFF;  // AD5680 18-bit
static const uint16_t ADC_MAX_CODE = 0xFFFF;   // AD7694 16-bit

// ======================================================
// Timing
// ======================================================
static const uint32_t DAC_SETTLE_US       = 5000;    // analog settle after DAC update
static const uint32_t ADC_CONVERSION_US   = 6;       // AD7694 conversion wait
static const uint32_t ADC_BETWEEN_AVG_US  = 100;     // gap between averaged ADC reads
static const uint8_t  ADC_AVG_SAMPLES     = 4;
static const uint32_t SWEEP_INTERVAL_MS   = 3000;

// ======================================================
// Auto-range policy for LMP91000 TIA
// ======================================================
// LMP91000 TIACN.TIA_GAIN code:
// 0 = external RTIA, 1 = 2.75k, 2 = 3.5k, 3 = 7k,
// 4 = 14k, 5 = 35k, 6 = 120k, 7 = 350k.
// We keep RLOAD = 10 ohm, same as your original TIACN=0x14.
#define ENABLE_AUTO_RANGE 1

static const uint8_t LMP_RLOAD_BITS = 0x00;   // 10 ohm
static const float ADC_ZERO_VOLTAGE = ADC_VREF * 0.5f; // REFCN INT_Z=50%, expected ~2.048V

// Change gain only when the ADC output is too close to rail or too small.
// Values are intentionally conservative to avoid gain hunting.
static const float AUTO_RANGE_HIGH_ABS_V = 1.75f; // |ADC - 2.048| > 1.75V => lower gain
static const float AUTO_RANGE_LOW_ABS_V  = 0.12f; // |ADC - 2.048| < 0.12V => higher gain
static const uint8_t AUTO_RANGE_CONFIRM_COUNT = 3;
static const uint32_t AUTO_RANGE_SETTLE_US = 5000;

struct TiaRange {
  uint8_t gainCode;
  float rtiaOhm;
  const char *name;
};

static const TiaRange tiaRanges[] = {
  {1,   2750.0f, "2.75k"},
  {2,   3500.0f, "3.5k"},
  {3,   7000.0f, "7k"},
  {4,  14000.0f, "14k"},
  {5,  35000.0f, "35k"},
  {6, 120000.0f, "120k"},
  {7, 350000.0f, "350k"}
};

static const int8_t TIA_RANGE_COUNT = sizeof(tiaRanges) / sizeof(tiaRanges[0]);
static int8_t currentTiaRangeIndex = 4; // default 35k, same as original TIACN=0x14
static uint8_t autoRangeLowCount = 0;
static uint8_t autoRangeHighCount = 0;

// ======================================================
// Shared Hardware SPI + LovyanGFX
// ======================================================
// ใช้ Hardware SPI bus เดียวกันสำหรับ TFT, DAC, ADC
// สำคัญ: ทุก transaction ต้องถือ spiMutex และ deselect chip อื่นก่อนเสมอ
SPIClass spiBus(FSPI);   // ESP32-S3: FSPI usually maps to SPI2_HOST / SPI2_HOST in LovyanGFX

// เริ่ม conservative ก่อน ถ้านิ่งแล้วค่อยเพิ่มความเร็ว
SPISettings dacSPI(1000000, MSBFIRST, SPI_MODE1);  // AD5680: ถ้า DAC ไม่ถูกลอง SPI_MODE0
SPISettings adcSPI(1000000, MSBFIRST, SPI_MODE0);  // AD7694: เริ่ม 1 MHz ก่อน

void deselectAllSPI() {
#if ENABLE_TFT_DISPLAY
  digitalWrite(TFT_CS, HIGH);
#endif
#if ENABLE_TOUCH_CONTROL
  digitalWrite(TOUCH_CS, HIGH);
#endif
  digitalWrite(DAC_SYNC, HIGH);
  digitalWrite(ADC_CNV, HIGH); // AD7694 deselect/tri-state MISO while sharing SPI
}

#if ENABLE_TFT_DISPLAY
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488 _panel_instance;
  lgfx::Bus_SPI       _bus_instance;
#if ENABLE_TOUCH_CONTROL
  lgfx::Touch_XPT2046 _touch_instance;
#endif

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 20000000;       // Safer for shared SPI with TFT + Touch + DAC/ADC
      cfg.freq_read   = 8000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = 0;              // Disable DMA first; safer when several devices share SPI2_HOST

      cfg.pin_sclk = SPI_SCK;
      cfg.pin_mosi = SPI_MOSI;
      cfg.pin_miso = SPI_MISO;
      cfg.pin_dc   = TFT_DC;

      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = TFT_CS;
      cfg.pin_rst          = TFT_RST;
      cfg.pin_busy         = -1;
      cfg.memory_width     = 320;
      cfg.memory_height    = 480;
      cfg.panel_width      = 320;
      cfg.panel_height     = 480;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = true;
      cfg.invert           = false;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true;      // สำคัญ: bus นี้แชร์กับ DAC/ADC

      _panel_instance.config(cfg);
    }

#if ENABLE_TOUCH_CONTROL
    {
      auto cfg = _touch_instance.config();

      // Same touch configuration as GLCDtouchMSP3520LOVYdriver.cpp
      // This board responds correctly when LovyanGFX uses the XPT2046 IRQ pin.
      cfg.x_min = 0;
      cfg.x_max = 4095;
      cfg.y_min = 0;
      cfg.y_max = 4095;

      cfg.pin_int  = TOUCH_IRQ;
      cfg.bus_shared = true;
      cfg.offset_rotation = 0;

      cfg.spi_host = SPI2_HOST;
      cfg.freq     = 1000000;

      cfg.pin_sclk = TOUCH_CLK;
      cfg.pin_mosi = TOUCH_MOSI;
      cfg.pin_miso = TOUCH_MISO;
      cfg.pin_cs   = TOUCH_CS;

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);
    }
#endif

    setPanel(&_panel_instance);
  }
};

LGFX lcd;

// TFT button geometry. Keep these values matched with drawRunButton().
static const int16_t BTN_X = 350;
static const int16_t BTN_Y = 10;
static const int16_t BTN_W = 110;
static const int16_t BTN_H = 45;
#endif

// ======================================================
// LMP91000 register
// ======================================================
#define LMP91000_ADDR 0x48

#define REG_STATUS 0x00
#define REG_LOCK   0x01
#define REG_TIACN  0x10
#define REG_REFCN  0x11
#define REG_MODECN 0x12

// ======================================================
// FreeRTOS objects
// ======================================================
static SemaphoreHandle_t spiMutex = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;

static TaskHandle_t measurementTaskHandle = nullptr;
static TaskHandle_t optionTaskHandle = nullptr;

// ======================================================
// Task priority policy
// ======================================================
// Measurement ไม่ควรใช้ priority สูงสุดตลอดเวลา
// - STOP / idle: priority ต่ำ เพื่อให้ GLCD/Touch/Serial ทำงานลื่น
// - RUN / measuring: priority สูงขึ้นเฉพาะช่วง sweep
// หมายเหตุ: measurementTask อยู่ Core 1, optionTask อยู่ Core 0
// priority จึงมีผลหลักกับ task อื่นบน core เดียวกัน และพฤติกรรมการรอ mutex
static const UBaseType_t MEAS_PRIORITY_IDLE = 1;
static const UBaseType_t MEAS_PRIORITY_RUN  = 5;
static const UBaseType_t OPTION_PRIORITY    = 2;

static QueueHandle_t loggerQueue = nullptr;

// ======================================================
// Runtime state
// ======================================================
volatile bool measurementEnabled = false;
volatile bool measurementBusy = false;
volatile bool measurementStopRequested = true;
volatile bool measurementPriorityIsHigh = false;

void setMeasurementPriority(bool high) {
  if (!measurementTaskHandle) return;

  if (measurementPriorityIsHigh == high) return;
  measurementPriorityIsHigh = high;

  vTaskPrioritySet(
    measurementTaskHandle,
    high ? MEAS_PRIORITY_RUN : MEAS_PRIORITY_IDLE
  );
}

void requestMeasurementRun() {
  // Wake MeasurementTask only when it is intentionally requested.
  // When idle, MeasurementTask is blocked in ulTaskNotifyTake() and uses no CPU time.
  measurementStopRequested = false;
  measurementEnabled = true;

  setMeasurementPriority(true);

  if (measurementTaskHandle != nullptr) {
    xTaskNotifyGive(measurementTaskHandle);
  }
}

void requestMeasurementStop() {
  // Do not forcibly suspend the task. Let it leave the measurement loop
  // at a safe point after the current ADC/DAC transaction.
  measurementStopRequested = true;
  measurementEnabled = false;

  // Lower priority immediately so UI/Serial/Touch stay responsive while
  // MeasurementTask finishes the current point and returns to blocked state.
  setMeasurementPriority(false);
}

// ======================================================
// Data structures
// ======================================================
enum SweepDirection : int8_t {
  DIR_CENTER = 0,
  DIR_UP     = 1,
  DIR_DOWN   = -1
};

struct MeasurementData {
  uint32_t sampleIndex;
  SweepDirection direction;
  float dacSetVoltage;
  uint32_t dacCode;
  float dacExpectedVoltage;
  uint16_t adcRaw;
  float adcVoltage;
  float adcDeltaVoltage;
  float currentAmp;
  float rtiaOhm;
  uint8_t tiaCN;
  char autoRangeAction; // '-' no change, 'U' gain up, 'D' gain down
  uint64_t timestamp_us;
  uint32_t period_us;
};

// Latest measurement for manual TFT refresh.
static MeasurementData lastMeasurement;
volatile bool hasLastMeasurement = false;

// Centralized TFT refresh requests.
// Only optionTask executes the actual lcd.* drawing.
// Touch/Serial/Web callbacks should request a refresh instead of drawing directly.
volatile bool tftFullRefreshRequested = false;
volatile bool tftRunButtonRefreshRequested = false;
volatile bool tftDrawMeasurementRequested = false;

void requestTftFullRefresh() {
  tftFullRefreshRequested = true;
  tftRunButtonRefreshRequested = true;
  if (hasLastMeasurement) {
    tftDrawMeasurementRequested = true;
  }
}

void requestTftRunButtonRefresh() {
  tftRunButtonRefreshRequested = true;
}

void requestTftMeasurementRefresh() {
  if (hasLastMeasurement) {
    tftDrawMeasurementRequested = true;
  }
}

#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
static uint16_t touchCalData[8] = {0};
static bool touchCalLoaded = false;
volatile bool touchCalibrationMode = false;
#endif

// Queue diagnostics
volatile uint32_t loggerDropCount = 0;

// ======================================================
// Utility
// ======================================================
static const char *directionToText(SweepDirection d) {
  if (d == DIR_UP) return "UP";
  if (d == DIR_DOWN) return "DOWN";
  return "CENTER";
}

void printHex8(const char *name, uint8_t value) {
  Serial.print(name);
  Serial.print(" = 0x");
  if (value < 0x10) Serial.print("0");
  Serial.println(value, HEX);
}

uint8_t makeTIACN(uint8_t gainCode) {
  return (uint8_t)(((gainCode & 0x07) << 2) | (LMP_RLOAD_BITS & 0x03));
}

float getCurrentRtiaOhm() {
  int8_t idx = currentTiaRangeIndex;
  if (idx < 0) idx = 0;
  if (idx >= TIA_RANGE_COUNT) idx = TIA_RANGE_COUNT - 1;
  return tiaRanges[idx].rtiaOhm;
}

uint8_t getCurrentTIACN() {
  int8_t idx = currentTiaRangeIndex;
  if (idx < 0) idx = 0;
  if (idx >= TIA_RANGE_COUNT) idx = TIA_RANGE_COUNT - 1;
  return makeTIACN(tiaRanges[idx].gainCode);
}

const char *getCurrentRangeName() {
  int8_t idx = currentTiaRangeIndex;
  if (idx < 0) idx = 0;
  if (idx >= TIA_RANGE_COUNT) idx = TIA_RANGE_COUNT - 1;
  return tiaRanges[idx].name;
}

// ======================================================
// LMP91000 low-level
// ======================================================
void lmpEnable() {
  if (LMP_MENB_PIN >= 0) {
    digitalWrite(LMP_MENB_PIN, LOW);   // MENB active low
    delayMicroseconds(10);
  }
}

bool lmpWriteReg_safe(uint8_t reg, uint8_t value) {
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);

  lmpEnable();

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();

  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return err == 0;
}

uint8_t lmpReadReg_safe(uint8_t reg) {
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);

  lmpEnable();

  Wire.beginTransmission(LMP91000_ADDR);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);

  if (err != 0) {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return 0xFF;
  }

  uint8_t n = Wire.requestFrom(LMP91000_ADDR, (uint8_t)1);

  uint8_t value = 0xFF;
  if (n == 1 && Wire.available()) {
    value = Wire.read();
  }

  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return value;
}

bool lmpInitFromYourCode() {
  uint8_t status = 0x00;

  Serial.println("Waiting LMP91000 ready...");

  for (int i = 0; i < 20; i++) {
    status = lmpReadReg_safe(REG_STATUS);

    Serial.print("STATUS try ");
    Serial.print(i);
    Serial.print(" = 0x");
    Serial.println(status, HEX);

    if (status & 0x01) {
      break;
    }

    delay(10);
  }

  if ((status & 0x01) == 0) {
    Serial.println("LMP91000 not ready.");
    return false;
  }

  // Unlock TIACN / REFCN
  if (!lmpWriteReg_safe(REG_LOCK, 0x00)) {
    Serial.println("Failed to unlock LMP91000.");
    return false;
  }

  delay(1);

  // Default TIACN = 35k, RLOAD = 10 ohm ตาม code เดิม
  currentTiaRangeIndex = 4; // 35k
  if (!lmpWriteReg_safe(REG_TIACN, getCurrentTIACN())) {
    Serial.println("Failed to write TIACN.");
    return false;
  }

  delay(1);

  // REFCN = 0xE0
  // External REF, INT_Z = 50%, BIAS setting ตาม code เดิม
  if (!lmpWriteReg_safe(REG_REFCN, 0xE0)) {
    Serial.println("Failed to write REFCN.");
    return false;
  }

  delay(1);

  // MODECN = 0x01
  // ใช้ค่าตาม code เดิม
  if (!lmpWriteReg_safe(REG_MODECN, 0x01)) {
    Serial.println("Failed to write MODECN.");
    return false;
  }

  delay(1);

  // Lock
  if (!lmpWriteReg_safe(REG_LOCK, 0x01)) {
    Serial.println("Failed to lock LMP91000.");
    return false;
  }

  delay(1);

  return true;
}

void printLMPRegisters_safe() {
  Serial.println("LMP91000 Registers:");
  printHex8("STATUS", lmpReadReg_safe(REG_STATUS));
  printHex8("LOCK  ", lmpReadReg_safe(REG_LOCK));
  printHex8("TIACN ", lmpReadReg_safe(REG_TIACN));
  printHex8("REFCN ", lmpReadReg_safe(REG_REFCN));
  printHex8("MODECN", lmpReadReg_safe(REG_MODECN));
}

// ======================================================
// Shared SPI helpers
// ======================================================
static inline bool takeSPI(uint32_t timeoutMs = 500) {
  if (!spiMutex) return true;
  return xSemaphoreTake(spiMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

static inline void giveSPI() {
  if (spiMutex) xSemaphoreGive(spiMutex);
}

// ======================================================
// AD5680 DAC
// ======================================================
uint32_t voltageToDACCode(float voltage) {
  if (voltage <= 0.0f) return 0;
  if (voltage >= DAC_VREF) return DAC_MAX_CODE;
  return (uint32_t)roundf((voltage / DAC_VREF) * DAC_MAX_CODE);
}

float dacCodeToVoltage(uint32_t code18) {
  code18 &= DAC_MAX_CODE;
  return ((float)code18 * DAC_VREF) / (float)DAC_MAX_CODE;
}

void writeAD5680_safe(uint32_t code18) {
  code18 &= DAC_MAX_CODE;

  // AD5680 24-bit frame: DB19..DB2 = 18-bit data
  uint32_t frame = code18 << 2;
  uint8_t b1 = (frame >> 16) & 0xFF;
  uint8_t b2 = (frame >> 8)  & 0xFF;
  uint8_t b3 = frame & 0xFF;

  if (!takeSPI(500)) {
    Serial.println("ERR: DAC waiting spiMutex timeout");
    return;
  }

  deselectAllSPI();
  spiBus.beginTransaction(dacSPI);

  digitalWrite(DAC_SYNC, LOW);
  delayMicroseconds(1);

  spiBus.transfer(b1);
  spiBus.transfer(b2);
  spiBus.transfer(b3);

  delayMicroseconds(1);
  digitalWrite(DAC_SYNC, HIGH);
  spiBus.endTransaction();

  giveSPI();

  // AD5680 + analog path settle, short internal settle only
  delayMicroseconds(50);
}


// ======================================================
// AD7694 ADC
// ======================================================
float adcRawToVoltage(uint16_t raw) {
  return ((float)raw * ADC_VREF) / 65535.0f;
}

static inline uint16_t readAD7694Raw_transaction_locked() {
  // Must be called while holding spiMutex.
  // Important for shared SPI:
  // AD7694 SDO shares MISO with XPT2046 touch. Leaving CNV LOW after a read can
  // keep the ADC serial output active and block the touch controller on MISO.
  // Therefore CNV is kept HIGH when idle, and returned HIGH after every read.
  uint16_t raw = 0;

  deselectAllSPI();              // leaves ADC_CNV HIGH, TFT/Touch/DAC deselected
  delayMicroseconds(1);

  // Generate a clean low -> high edge to start a fresh conversion.
  digitalWrite(ADC_CNV, LOW);
  delayMicroseconds(1);
  digitalWrite(ADC_CNV, HIGH);
  delayMicroseconds(ADC_CONVERSION_US);

  // CNV LOW enables serial data output for this read only.
  digitalWrite(ADC_CNV, LOW);
  delayMicroseconds(1);

  spiBus.beginTransaction(adcSPI);
  uint8_t hi = spiBus.transfer(0x00);
  uint8_t lo = spiBus.transfer(0x00);
  spiBus.endTransaction();

  // Release ADC SDO/MISO immediately so TFT/Touch can use the shared bus.
  digitalWrite(ADC_CNV, HIGH);

  raw = ((uint16_t)hi << 8) | lo;
  return raw;
}

uint16_t readAD7694Raw_safe() {
  if (!takeSPI(500)) {
    Serial.println("ERR: ADC waiting spiMutex timeout");
    return 0;
  }

  uint16_t raw = readAD7694Raw_transaction_locked();
  giveSPI();
  return raw;
}

uint16_t readAD7694Average_safe(uint8_t samples) {
  if (samples == 0) samples = 1;

  if (!takeSPI(500)) {
    Serial.println("ERR: ADC average waiting spiMutex timeout");
    return 0;
  }

  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += readAD7694Raw_transaction_locked();
    if (i + 1 < samples) {
      delayMicroseconds(ADC_BETWEEN_AVG_US);
    }
  }

  giveSPI();
  return (uint16_t)(sum / samples);
}

// ======================================================
// Auto-range helpers
// ======================================================
bool setTiaRangeIndex_safe(int8_t newIndex) {
  if (newIndex < 0) newIndex = 0;
  if (newIndex >= TIA_RANGE_COUNT) newIndex = TIA_RANGE_COUNT - 1;
  if (newIndex == currentTiaRangeIndex) return true;

  uint8_t newTIACN = makeTIACN(tiaRanges[newIndex].gainCode);

  bool okUnlock = lmpWriteReg_safe(REG_LOCK, 0x00);
  bool okWrite  = lmpWriteReg_safe(REG_TIACN, newTIACN);
  bool okLock   = lmpWriteReg_safe(REG_LOCK, 0x01);

  if (okUnlock && okWrite && okLock) {
    currentTiaRangeIndex = newIndex;
    delayMicroseconds(AUTO_RANGE_SETTLE_US);
    return true;
  }

  Serial.println("ERR: auto range failed to write LMP91000 TIACN");
  return false;
}

char autoRangeUpdateAndMaybeChange(float adcVoltage) {
#if ENABLE_AUTO_RANGE
  float absDelta = fabsf(adcVoltage - ADC_ZERO_VOLTAGE);

  // Output too close to rail: lower gain immediately after confirmation.
  if (absDelta > AUTO_RANGE_HIGH_ABS_V) {
    autoRangeHighCount++;
    autoRangeLowCount = 0;

    if (autoRangeHighCount >= AUTO_RANGE_CONFIRM_COUNT && currentTiaRangeIndex > 0) {
      autoRangeHighCount = 0;
      if (setTiaRangeIndex_safe(currentTiaRangeIndex - 1)) {
        return 'D'; // Down gain / lower RTIA
      }
    }
    return '-';
  }

  // Output is very small around zero: increase gain after confirmation.
  if (absDelta < AUTO_RANGE_LOW_ABS_V) {
    autoRangeLowCount++;
    autoRangeHighCount = 0;

    if (autoRangeLowCount >= AUTO_RANGE_CONFIRM_COUNT && currentTiaRangeIndex < (TIA_RANGE_COUNT - 1)) {
      autoRangeLowCount = 0;
      if (setTiaRangeIndex_safe(currentTiaRangeIndex + 1)) {
        return 'U'; // Up gain / higher RTIA
      }
    }
    return '-';
  }

  autoRangeLowCount = 0;
  autoRangeHighCount = 0;
#endif
  return '-';
}

void fillCurrentFields(MeasurementData &data) {
  data.adcDeltaVoltage = data.adcVoltage - ADC_ZERO_VOLTAGE;
  data.rtiaOhm = getCurrentRtiaOhm();
  data.tiaCN = getCurrentTIACN();

  // Polarity may depend on your electrochemical connection.
  // For LMP91000 TIA output, this gives the measured transimpedance relation.
  data.currentAmp = data.adcDeltaVoltage / data.rtiaOhm;
}

// ======================================================
// Push measurement to queues
// ======================================================
void publishMeasurement(const MeasurementData &data) {
  // Keep latest value for manual display refresh.
  lastMeasurement = data;
  hasLastMeasurement = true;

  // Logger queue keeps history. Drop if full to avoid blocking measurement.
  if (loggerQueue) {
    if (xQueueSend(loggerQueue, &data, 0) != pdTRUE) {
      loggerDropCount++;
    }
  }
}

// ======================================================
// Single measurement point
// ======================================================
void measureOnePoint(float dacSetVoltage, SweepDirection direction, uint32_t &sampleIndex) {
  MeasurementData data;

  uint64_t t0 = esp_timer_get_time();

  measurementBusy = true;

  data.sampleIndex = sampleIndex++;
  data.direction = direction;
  data.dacSetVoltage = dacSetVoltage;
  data.dacCode = voltageToDACCode(dacSetVoltage);
  data.dacExpectedVoltage = dacCodeToVoltage(data.dacCode);
  data.autoRangeAction = '-';

  writeAD5680_safe(data.dacCode);

  // Main analog settling time. Keep this in measurement task.
  delayMicroseconds(DAC_SETTLE_US);

  data.adcRaw = readAD7694Average_safe(ADC_AVG_SAMPLES);
  data.adcVoltage = adcRawToVoltage(data.adcRaw);

  // Auto-range decision is made after the first read.
  // If gain changes, re-read the same DAC point so the logged sample uses the new range.
  char action = autoRangeUpdateAndMaybeChange(data.adcVoltage);
  if (action != '-') {
    data.autoRangeAction = action;
    data.adcRaw = readAD7694Average_safe(ADC_AVG_SAMPLES);
    data.adcVoltage = adcRawToVoltage(data.adcRaw);
  }

  fillCurrentFields(data);

  data.timestamp_us = esp_timer_get_time();
  data.period_us = (uint32_t)(data.timestamp_us - t0);

  measurementBusy = false;

  publishMeasurement(data);
}

// ======================================================
// Tasks
// ======================================================
void measurementTask(void *pvParameters) {
  Serial.println("MeasurementTask started on Core " + String(xPortGetCoreID()));
  uint32_t sampleIndex = 0;

  // Set DAC to 0V once during startup.
  writeAD5680_safe(0);
  vTaskDelay(pdMS_TO_TICKS(500));

  for (;;) {
    // Idle state:
    // MeasurementTask sleeps here and consumes no CPU until requestMeasurementRun()
    // calls xTaskNotifyGive(measurementTaskHandle).
    measurementEnabled = false;
    measurementBusy = false;
    measurementStopRequested = true;
    setMeasurementPriority(false);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // A stale notification may arrive after STOP or during startup. Ignore it.
    if (!measurementEnabled) {
      continue;
    }

    measurementStopRequested = false;
    setMeasurementPriority(true);

    Serial.println("Measurement started");

    while (measurementEnabled && !measurementStopRequested) {
      Serial.println("Measurement sweep start");

      // Sweep up: 0.000V to 4.090V, step 10mV
      for (int mv = 0; mv <= 4090; mv += 10) {
        if (!measurementEnabled || measurementStopRequested) break;
        measureOnePoint(mv / 1000.0f, DIR_UP, sampleIndex);
        vTaskDelay(1);
      }

      // Full-scale point 4.096V
      if (measurementEnabled && !measurementStopRequested) {
        measureOnePoint(4.096f, DIR_UP, sampleIndex);
        vTaskDelay(1);
      }

      // Sweep down: 4.090V to 0.000V, step 10mV
      for (int mv = 4090; mv >= 0; mv -= 10) {
        if (!measurementEnabled || measurementStopRequested) break;
        measureOnePoint(mv / 1000.0f, DIR_DOWN, sampleIndex);
        vTaskDelay(1);
      }

      // Center point
      if (measurementEnabled && !measurementStopRequested) {
        measureOnePoint(2.048f, DIR_CENTER, sampleIndex);
      }

      Serial.println("Measurement sweep done");

      // Lower priority while waiting between sweeps. If STOP is requested during
      // this interval, the task exits quickly instead of sleeping for the full time.
      setMeasurementPriority(false);

      uint32_t waitedMs = 0;
      while (measurementEnabled && !measurementStopRequested && waitedMs < SWEEP_INTERVAL_MS) {
        vTaskDelay(pdMS_TO_TICKS(20));
        waitedMs += 20;
      }

      if (measurementEnabled && !measurementStopRequested) {
        setMeasurementPriority(true);
      }
    }

    measurementBusy = false;
    measurementEnabled = false;
    measurementStopRequested = true;
    setMeasurementPriority(false);

    Serial.println("Measurement stopped, task blocked until next RUN");
  }
}

#if ENABLE_TFT_DISPLAY
void drawStaticScreen() {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.startWrite();
  lcd.setRotation(1);                 // Set rotation before clearing/drawing.
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("smart-ec");

  lcd.setTextSize(1);
  lcd.setCursor(10, 42);
  lcd.println("LovyanGFX + Shared HW SPI");
  lcd.setCursor(10, 58);
  lcd.println("Core1: Measurement  Core0: Option/UI");

  // Start/Stop button area
  lcd.drawRect(BTN_X, BTN_Y, BTN_W, BTN_H, TFT_WHITE);

  lcd.endWrite();
  deselectAllSPI();
}

void drawRunButton(bool running) {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.startWrite();
  uint16_t bg = running ? TFT_RED : TFT_GREEN;
  lcd.fillRect(BTN_X + 2, BTN_Y + 2, BTN_W - 4, BTN_H - 4, bg);
  lcd.setTextColor(TFT_BLACK, bg);
  lcd.setTextSize(2);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString(running ? "STOP" : "RUN", BTN_X + BTN_W / 2, BTN_Y + BTN_H / 2);
  lcd.setTextDatum(TL_DATUM);
  lcd.endWrite();

  deselectAllSPI();
}

void drawMeasurement(const MeasurementData &data) {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.startWrite();
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextSize(2);

  lcd.fillRect(10, 80, 460, 230, TFT_BLACK);

  lcd.setCursor(10, 80);
  lcd.print("Dir: ");
  lcd.println(directionToText(data.direction));

  lcd.setCursor(10, 110);
  lcd.print("DAC set: ");
  lcd.print(data.dacSetVoltage, 3);
  lcd.println(" V");

  lcd.setCursor(10, 140);
  lcd.print("DAC code: ");
  lcd.println(data.dacCode);

  lcd.setCursor(10, 170);
  lcd.print("ADC raw: ");
  lcd.println(data.adcRaw);

  lcd.setCursor(10, 200);
  lcd.print("ADC V: ");
  lcd.print(data.adcVoltage, 6);
  lcd.println(" V");

  lcd.setCursor(10, 230);
  lcd.print("RTIA: ");
  lcd.print(data.rtiaOhm, 0);
  lcd.print(" ohm ");
  lcd.print(data.autoRangeAction);

  lcd.setCursor(10, 260);
  lcd.print("I: ");
  lcd.print(data.currentAmp * 1000000.0f, 3);
  lcd.println(" uA");

  lcd.setCursor(10, 290);
  lcd.print("Period: ");
  lcd.print(data.period_us);
  lcd.println(" us");

  lcd.endWrite();
  deselectAllSPI();
}

void serviceTftRefresh() {
  // Must be called only from optionTask.
  if (measurementBusy) return;

  bool doFull = tftFullRefreshRequested;
  bool doRun  = tftRunButtonRefreshRequested;
  bool doMeas = tftDrawMeasurementRequested && hasLastMeasurement;

  if (!doFull && !doRun && !doMeas) return;

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  // Clear requests after we get the mutex, so a missed draw can be requested again.
  tftFullRefreshRequested = false;
  tftRunButtonRefreshRequested = false;
  tftDrawMeasurementRequested = false;

  deselectAllSPI();

  if (doFull) {
    drawStaticScreen();
  }

  if (doRun || doFull) {
    drawRunButton(measurementEnabled);
  }

  if (doMeas) {
    MeasurementData snapshot = lastMeasurement;
    drawMeasurement(snapshot);
  }

  deselectAllSPI();
  xSemaphoreGive(spiMutex);
}
#else
void serviceTftRefresh() {}
#endif

// ======================================================
// Option task helpers: Serial / logger / I2C / manual TFT
// ======================================================
void printMeasurementCsv(const MeasurementData &data) {
  Serial.print(directionToText(data.direction));
  Serial.print(',');
  Serial.print(data.dacSetVoltage, 3);
  Serial.print(',');
  Serial.print(data.dacCode);
  Serial.print(',');
  Serial.print(data.dacExpectedVoltage, 6);
  Serial.print(',');
  Serial.print(data.adcRaw);
  Serial.print(',');
  Serial.print(data.adcVoltage, 6);
  Serial.print(',');
  Serial.print(data.adcDeltaVoltage, 6);
  Serial.print(',');
  Serial.print(data.currentAmp, 12);
  Serial.print(',');
  Serial.print(data.rtiaOhm, 0);
  Serial.print(',');
  Serial.print("0x");
  if (data.tiaCN < 0x10) Serial.print("0");
  Serial.print(data.tiaCN, HEX);
  Serial.print(',');
  Serial.print(data.autoRangeAction);
  Serial.print(',');
  Serial.print((unsigned long long)data.timestamp_us);
  Serial.print(',');
  Serial.println(data.period_us);
}

bool waitMeasurementIdle(uint32_t timeoutMs) {
  uint32_t startMs = millis();
  while (measurementBusy) {
    if (millis() - startMs >= timeoutMs) return false;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return true;
}

void setLMPRegisterOption(uint8_t reg, uint8_t value) {
  if (measurementEnabled) {
    Serial.println("I2C write skipped: STOP measurement first");
    return;
  }

  if (!waitMeasurementIdle(500)) {
    Serial.println("I2C write skipped: measurement busy timeout");
    return;
  }

  bool okUnlock = lmpWriteReg_safe(REG_LOCK, 0x00);
  bool okWrite  = lmpWriteReg_safe(reg, value);
  bool okLock   = lmpWriteReg_safe(REG_LOCK, 0x01);

  Serial.print("I2C SET REG 0x");
  Serial.print(reg, HEX);
  Serial.print(" = 0x");
  Serial.print(value, HEX);
  Serial.print(" result=");
  Serial.println((okUnlock && okWrite && okLock) ? "OK" : "FAIL");
}

#if ENABLE_TFT_DISPLAY
void manualRefreshDisplay() {
  if (measurementEnabled || measurementBusy) {
    Serial.println("TFT refresh skipped: STOP measurement first");
    return;
  }

  if (!waitMeasurementIdle(500)) {
    Serial.println("TFT refresh skipped: measurement busy timeout");
    return;
  }

  requestTftFullRefresh();
  Serial.println("TFT refresh requested");
}
#else
void manualRefreshDisplay() {
  Serial.println("TFT disabled");
}
#endif

void refreshRunButtonFromOptionTask() {
#if ENABLE_TFT_DISPLAY
  requestTftRunButtonRefresh();
#endif
}

void handleTouchControl() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL && ENABLE_TOUCH_POLLING
  static uint32_t lastTouchMs = 0;
  static bool lastPressed = false;

  if (touchCalibrationMode) return;

  uint32_t nowMs = millis();
  if (nowMs - lastTouchMs < 50) return;   // about 20 Hz
  lastTouchMs = nowMs;

  // Do not read touch during the exact measurement transaction window.
  // This lets STOP remain responsive between measurement points/sweeps.
  if (measurementBusy) return;

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(5)) != pdTRUE) {
    return;
  }

  deselectAllSPI();

  uint16_t x = 0;
  uint16_t y = 0;
  bool pressed = lcd.getTouch(&x, &y);

  deselectAllSPI();
  xSemaphoreGive(spiMutex);

  if (pressed && !lastPressed) {
    Serial.print("Touch x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.println(y);

    if (x >= BTN_X && x < BTN_X + BTN_W &&
        y >= BTN_Y && y < BTN_Y + BTN_H) {

      if (measurementEnabled) {
        requestMeasurementStop();
        digitalWrite(LED_BUILTIN_PIN, HIGH);
        Serial.println("Measurement STOP by touch");
      } else {
        requestMeasurementRun();
        digitalWrite(LED_BUILTIN_PIN, LOW);
        Serial.println("Measurement RUN by touch");
      }

      refreshRunButtonFromOptionTask();
    }
  }

  lastPressed = pressed;
#endif
}
bool saveTouchCalibration(const uint16_t *data) {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  File f = LittleFS.open(CALIBRATION_FILE, "w");
  if (!f) {
    Serial.println("Touch cal save failed: cannot open file");
    return false;
  }

  size_t written = f.write((const uint8_t *)data, sizeof(uint16_t) * 8);
  f.close();

  if (written != sizeof(uint16_t) * 8) {
    Serial.println("Touch cal save failed: wrong byte count");
    return false;
  }

  Serial.print("Touch calibration saved to ");
  Serial.println(CALIBRATION_FILE);
  return true;
#else
  return false;
#endif
}

bool loadTouchCalibration() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  if (!LittleFS.exists(CALIBRATION_FILE)) {
    Serial.println("No touch calibration file found.");
    return false;
  }

  File f = LittleFS.open(CALIBRATION_FILE, "r");
  if (!f) {
    Serial.println("Touch cal load failed: cannot open file");
    return false;
  }

  if (f.size() != sizeof(uint16_t) * 8) {
    Serial.print("Touch cal load failed: invalid file size = ");
    Serial.println(f.size());
    f.close();
    return false;
  }

  size_t n = f.readBytes((char *)touchCalData, sizeof(uint16_t) * 8);
  f.close();

  if (n != sizeof(uint16_t) * 8) {
    Serial.println("Touch cal load failed: read size mismatch");
    return false;
  }

  lcd.setTouchCalibrate(touchCalData);
  touchCalLoaded = true;

  Serial.print("Touch calibration loaded: { ");
  for (int i = 0; i < 8; i++) {
    Serial.print(touchCalData[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println(" }");

  return true;
#else
  return false;
#endif
}

void deleteTouchCalibration() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  if (LittleFS.exists(CALIBRATION_FILE)) {
    LittleFS.remove(CALIBRATION_FILE);
    Serial.println("Touch calibration file deleted.");
  } else {
    Serial.println("No touch calibration file to delete.");
  }
  touchCalLoaded = false;
#endif
}

void calibrateTouchAndPrint() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  Serial.println("Touch calibration requested.");
  Serial.println("Stopping measurement before calibration...");

  requestMeasurementStop();
  waitMeasurementIdle(1500);
  touchCalibrationMode = true;

  uint16_t calData[8] = {0};

  // Important:
  // Do NOT hold the external spiMutex during lcd.calibrateTouch().
  // The standalone working driver calls calibrateTouch() directly, and LovyanGFX
  // manages its own SPI lock internally. Holding our app mutex here can make the
  // touch flow behave differently from the proven working example.
  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    deselectAllSPI();
    lcd.setRotation(1);
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(TL_DATUM);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setCursor(20, 20);
    lcd.println("Touch calibration");
    lcd.println("Touch corners as shown");
    xSemaphoreGive(spiMutex);
  } else {
    Serial.println("Warning: spiMutex timeout before calibration screen draw");
  }

  delay(1000);

  // LovyanGFX in this environment returns void and fills calData[8].
  // This call now matches GLCDtouchMSP3520LOVYdriver.cpp as closely as possible.
  lcd.calibrateTouch(
    calData,
    TFT_MAGENTA,
    TFT_BLACK,
    max(lcd.width(), lcd.height()) >> 3
  );

  touchCalibrationMode = false;

  memcpy(touchCalData, calData, sizeof(touchCalData));
  lcd.setTouchCalibrate(touchCalData);
  touchCalLoaded = true;
  saveTouchCalibration(touchCalData);

  Serial.println();
  Serial.println("Touch calibration data:");
  Serial.print("{ ");
  for (int i = 0; i < 8; i++) {
    Serial.print(touchCalData[i]);
    if (i < 7) Serial.print(", ");
  }
  Serial.println(" }; ");

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    deselectAllSPI();
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setCursor(20, 20);
    lcd.println("Calibration OK");
    xSemaphoreGive(spiMutex);
  }

  delay(800);

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    drawStaticScreen();
    drawRunButton(measurementEnabled);
    xSemaphoreGive(spiMutex);
  }

  Serial.println("Calibration done.");
#endif
}

uint16_t xpt2046Read12_locked(uint8_t cmd) {
  // Must be called while holding spiMutex.
  // Common XPT2046 commands: 0xD0 = X, 0x90 = Y, 0xB0 = Z1, 0xC0 = Z2.
  deselectAllSPI();
  spiBus.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(TOUCH_CS, LOW);
  delayMicroseconds(2);
  spiBus.transfer(cmd);
  uint8_t hi = spiBus.transfer(0x00);
  uint8_t lo = spiBus.transfer(0x00);
  digitalWrite(TOUCH_CS, HIGH);
  spiBus.endTransaction();
  delayMicroseconds(2);
  return (uint16_t)((((uint16_t)hi << 8) | lo) >> 3) & 0x0FFF;
}

void printRawTouchOnce() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  if (measurementBusy) {
    Serial.println("Raw touch skipped: measurement busy");
    return;
  }

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("Raw touch failed: spiMutex timeout");
    return;
  }

  uint16_t z1 = xpt2046Read12_locked(0xB0);
  uint16_t z2 = xpt2046Read12_locked(0xC0);
  uint16_t x  = xpt2046Read12_locked(0xD0);
  uint16_t y  = xpt2046Read12_locked(0x90);
  deselectAllSPI();
  xSemaphoreGive(spiMutex);

  Serial.print("RAW XPT2046 x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.print(y);
  Serial.print(" z1=");
  Serial.print(z1);
  Serial.print(" z2=");
  Serial.println(z2);
  Serial.println("Tip: while pressing the screen, x/y should change and z1/z2 should not stay all 0 or all 4095.");
#else
  Serial.println("Touch disabled.");
#endif
}

void printTouchOnce() {
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  if (measurementBusy) {
    Serial.println("Touch read skipped: measurement busy");
    return;
  }

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    Serial.println("Touch read failed: spiMutex timeout");
    return;
  }

  deselectAllSPI();
  uint16_t x = 0;
  uint16_t y = 0;
  bool pressed = lcd.getTouch(&x, &y);
  deselectAllSPI();

  xSemaphoreGive(spiMutex);

  Serial.print("Touch pressed=");
  Serial.print(pressed ? "true" : "false");
  Serial.print(" x=");
  Serial.print(x);
  Serial.print(" y=");
  Serial.println(y);
#else
  Serial.println("Touch disabled.");
#endif
}


void resetAutoRangeToLowestForNewRun() {
  // User requirement:
  // Every Serial command 'r' must start from the lowest internal RTIA range.
  // This makes each new run predictable, then auto-range can increase RTIA as needed.
  if (!waitMeasurementIdle(1000)) {
    Serial.println("Auto-range reset skipped: measurement busy timeout");
    return;
  }

  autoRangeLowCount = 0;
  autoRangeHighCount = 0;

  if (setTiaRangeIndex_safe(4)) {
    Serial.print("Auto-range start range reset to ");
    Serial.print(getCurrentRangeName());
    Serial.print(" RTIA=");
    Serial.print(getCurrentRtiaOhm(), 0);
    Serial.print(" TIACN=0x");
    Serial.println(getCurrentTIACN(), HEX);
  }
}

void handleSerialCommand(char c) {
  if (c == '\r' || c == '\n') return;

  if (c == 'r' || c == 'R') {
    if (measurementEnabled) {
      Serial.println("Measurement already running; send 's' first before starting a new 2.75k run");
      return;
    }

    resetAutoRangeToLowestForNewRun();
    requestMeasurementRun();
    digitalWrite(LED_BUILTIN_PIN, LOW);
    refreshRunButtonFromOptionTask();
    Serial.println("Measurement RUN from RTIA 2.75k");
  }
  else if (c == 's' || c == 'S') {
    requestMeasurementStop();
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    refreshRunButtonFromOptionTask();
    Serial.println("Measurement STOP");
  }
  else if (c == 'p' || c == 'P') {
    if (measurementEnabled) {
      Serial.println("Print LMP skipped: STOP measurement first");
    } else {
      printLMPRegisters_safe();
    }
  }
  else if (c == 'g' || c == 'G') {
    if (measurementEnabled) {
      Serial.println("Set TIA range skipped: STOP measurement first");
    } else {
      setTiaRangeIndex_safe(4);
      Serial.println("TIA range set to 35k");
    }
  }
  else if (c == 'u' || c == 'U') {
    manualRefreshDisplay();
  }
  else if (c == 't' || c == 'T') {
    printTouchOnce();
  }
  else if (c == 'w' || c == 'W') {
    printRawTouchOnce();
  }
  else if (c == 'c' || c == 'C') {
    calibrateTouchAndPrint();
  }
  else if (c == 'x' || c == 'X') {
    deleteTouchCalibration();
  }
  else if (c == 'd' || c == 'D') {
    Serial.print("measurementEnabled=");
    Serial.println(measurementEnabled ? "true" : "false");
    Serial.print("measurementBusy=");
    Serial.println(measurementBusy ? "true" : "false");
    Serial.print("measurementPriorityIsHigh=");
    Serial.println(measurementPriorityIsHigh ? "true" : "false");
    Serial.print("loggerDropCount=");
    Serial.println(loggerDropCount);
    Serial.print("autoRange=");
    Serial.println(ENABLE_AUTO_RANGE ? "enabled" : "disabled");
    Serial.print("TIA range=");
    Serial.print(getCurrentRangeName());
    Serial.print(" RTIA=");
    Serial.print(getCurrentRtiaOhm(), 0);
    Serial.print(" TIACN=0x");
    Serial.println(getCurrentTIACN(), HEX);
    Serial.print("hasLastMeasurement=");
    Serial.println(hasLastMeasurement ? "true" : "false");
    Serial.print("ENABLE_TOUCH_CONTROL=");
    Serial.println(ENABLE_TOUCH_CONTROL);
    Serial.print("ENABLE_TOUCH_POLLING=");
    Serial.println(ENABLE_TOUCH_POLLING);
    Serial.print("TOUCH_CS=");
    Serial.println(TOUCH_CS);
    Serial.print("TOUCH_IRQ=");
    Serial.println(TOUCH_IRQ);
#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
    Serial.print("touchCalLoaded=");
    Serial.println(touchCalLoaded ? "true" : "false");
    Serial.print("CALIBRATION_FILE=");
    Serial.println(CALIBRATION_FILE);
#endif
  }
}

// WiFi Manager (runs from optionTask on Core 0)
// ======================================================
// Define these in platformio.ini build_flags or above this file if needed:
//   -D DEFAULT_WIFI_SSID=\"YourSSID\"
//   -D DEFAULT_WIFI_PASSWORD=\"YourPassword\"
//   -D DEFAULT_WIFI_USER=\"\"              // non-empty = WPA2 Enterprise
#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID ""
#endif
#ifndef DEFAULT_WIFI_PASSWORD
#define DEFAULT_WIFI_PASSWORD ""
#endif
#ifndef DEFAULT_WIFI_USER
#define DEFAULT_WIFI_USER ""
#endif

static const char *WIFI_NVS_NAMESPACE   = "smart_ec";
static const char *WIFI_NVS_KEY         = "wifi_list_v1";          // WiFi credentials are stored in NVS, not in LittleFS
static const char *WIFI_PORTAL_FILE     = "/wifi_index.html";      // put this file in PlatformIO data/ and upload LittleFS
static const char *WIFI_AP_PASSWORD     = "12345678";
static const uint8_t WIFI_MAX_CREDS     = 12;
static const uint32_t WIFI_DEFAULT_TIMEOUT_MS = 6000;
static const uint32_t WIFI_STORED_TIMEOUT_MS  = 4500;
static const uint32_t WIFI_RETRY_PERIOD_MS    = 30000;
static const uint16_t WIFI_SCAN_MS_PER_CH     = 120;  // active scan; all channels about 1.5-2.0s

struct WifiCredential {
  String ssid;
  String pass;
  String user;      // empty = WPA/WPA2 Personal, non-empty = WPA2 Enterprise username
  bool enterprise = false;
  int32_t channel = 0;
  uint8_t bssid[6] = {0, 0, 0, 0, 0, 0};
  bool hasBssid = false;
  int32_t lastRssi = -999;
};

static WifiCredential wifiCreds[WIFI_MAX_CREDS];
static uint8_t wifiCredCount = 0;
static WebServer wifiServer(80);
static DNSServer wifiDns;
static bool wifiServerStarted = false;
static bool wifiApMode = false;
static bool wifiConnectRequested = false;
static uint32_t wifiLastRetryMs = 0;
static String wifiApSsid = "smart-ec-Setup";

static String urlDecode(const String &src) {
  String out;
  out.reserve(src.length());
  for (size_t i = 0; i < src.length(); i++) {
    char c = src[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < src.length()) {
      char h1 = src[i + 1];
      char h2 = src[i + 2];
      auto hexVal = [](char x) -> int {
        if (x >= '0' && x <= '9') return x - '0';
        if (x >= 'A' && x <= 'F') return x - 'A' + 10;
        if (x >= 'a' && x <= 'f') return x - 'a' + 10;
        return -1;
      };
      int v1 = hexVal(h1);
      int v2 = hexVal(h2);
      if (v1 >= 0 && v2 >= 0) {
        out += char((v1 << 4) | v2);
        i += 2;
      } else {
        out += c;
      }
    } else {
      out += c;
    }
  }
  return out;
}

static String urlEncode(const String &src) {
  const char *hex = "0123456789ABCDEF";
  String out;
  out.reserve(src.length() * 3);
  for (size_t i = 0; i < src.length(); i++) {
    uint8_t c = (uint8_t)src[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
      out += char(c);
    } else if (c == ' ') {
      out += '+';
    } else {
      out += '%';
      out += hex[c >> 4];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

static bool parseBssid(const String &text, uint8_t out[6]) {
  if (text.length() != 17) return false;
  int values[6];
  if (sscanf(text.c_str(), "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)values[i];
  return true;
}

static String bssidToString(const uint8_t bssid[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
  return String(buf);
}

static String htmlEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else out += c;
  }
  return out;
}


static String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '\\') out += F("\\\\");
    else if (c == '"') out += F("\\\"");
    else if (c == '\n') out += F("\\n");
    else if (c == '\r') out += F("\\r");
    else if (c == '\t') out += F("\\t");
    else out += c;
  }
  return out;
}

static String serializeWifiCredentials() {
  String data;
  data.reserve(1024);

  for (uint8_t i = 0; i < wifiCredCount; i++) {
    WifiCredential &c = wifiCreds[i];
    data += urlEncode(c.ssid); data += '\t';
    data += urlEncode(c.pass); data += '\t';
    data += urlEncode(c.user); data += '\t';
    data += String(c.enterprise ? 1 : 0); data += '\t';
    data += String(c.channel); data += '\t';
    data += c.hasBssid ? bssidToString(c.bssid) : String("00:00:00:00:00:00");
    data += '\n';
  }
  return data;
}

static bool parseWifiCredentialsText(const String &data) {
  wifiCredCount = 0;
  int pos = 0;

  while (pos < (int)data.length() && wifiCredCount < WIFI_MAX_CREDS) {
    int nl = data.indexOf('\n', pos);
    if (nl < 0) nl = data.length();

    String line = data.substring(pos, nl);
    pos = nl + 1;
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;

    int p1 = line.indexOf('\t');
    int p2 = line.indexOf('\t', p1 + 1);
    int p3 = line.indexOf('\t', p2 + 1);
    int p4 = line.indexOf('\t', p3 + 1);
    int p5 = line.indexOf('\t', p4 + 1);
    if (p1 < 0 || p2 < 0 || p3 < 0 || p4 < 0 || p5 < 0) continue;

    WifiCredential &c = wifiCreds[wifiCredCount];
    c.ssid = urlDecode(line.substring(0, p1));
    c.pass = urlDecode(line.substring(p1 + 1, p2));
    c.user = urlDecode(line.substring(p2 + 1, p3));
    c.enterprise = line.substring(p3 + 1, p4).toInt() != 0;
    c.channel = line.substring(p4 + 1, p5).toInt();
    c.hasBssid = parseBssid(line.substring(p5 + 1), c.bssid);
    c.lastRssi = -999;

    if (c.ssid.length() > 0) wifiCredCount++;
  }

  return wifiCredCount > 0;
}

static bool loadWifiCredentials() {
  Preferences prefs;
  wifiCredCount = 0;

  if (!prefs.begin(WIFI_NVS_NAMESPACE, true)) {
    Serial.println("WiFi NVS open failed.");
    return false;
  }

  String data = prefs.getString(WIFI_NVS_KEY, "");
  prefs.end();

  if (data.length() == 0) {
    Serial.println("No WiFi credentials in NVS yet.");
    return false;
  }

  bool ok = parseWifiCredentialsText(data);
  Serial.print("Loaded WiFi credentials from NVS: ");
  Serial.println(wifiCredCount);
  return ok;
}

static bool saveWifiCredentials() {
  Preferences prefs;
  if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) {
    Serial.println("WiFi NVS write open failed.");
    return false;
  }

  String data = serializeWifiCredentials();
  size_t written = prefs.putString(WIFI_NVS_KEY, data);
  prefs.end();

  bool ok = written == data.length();
  if (!ok) {
    Serial.println("WiFi NVS write failed or truncated.");
  }
  return ok;
}

static void upsertWifiCredential(const WifiCredential &input) {
  if (input.ssid.length() == 0) return;

  for (uint8_t i = 0; i < wifiCredCount; i++) {
    if (wifiCreds[i].ssid == input.ssid && wifiCreds[i].user == input.user) {
      wifiCreds[i].pass = input.pass;
      wifiCreds[i].enterprise = input.enterprise;
      wifiCreds[i].channel = input.channel;
      wifiCreds[i].hasBssid = input.hasBssid;
      memcpy(wifiCreds[i].bssid, input.bssid, 6);
      saveWifiCredentials();
      return;
    }
  }

  if (wifiCredCount < WIFI_MAX_CREDS) {
    wifiCreds[wifiCredCount++] = input;
  } else {
    // If full, replace the last slot. This avoids dynamic allocation surprises.
    wifiCreds[WIFI_MAX_CREDS - 1] = input;
  }
  saveWifiCredentials();
}

static void deleteWifiCredential(uint8_t index) {
  if (index >= wifiCredCount) return;
  for (uint8_t i = index; i + 1 < wifiCredCount; i++) wifiCreds[i] = wifiCreds[i + 1];
  wifiCredCount--;
  saveWifiCredentials();
}

static void configureEnterpriseWifi(const WifiCredential &c) {
#if POTENV2_HAS_WPA2_ENTERPRISE
  esp_wifi_sta_wpa2_ent_disable();
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)c.user.c_str(), c.user.length());
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)c.user.c_str(), c.user.length());
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)c.pass.c_str(), c.pass.length());
  esp_wifi_sta_wpa2_ent_enable();
#else
  (void)c;
  Serial.println("WPA2 Enterprise is not available in this ESP32 Arduino core build.");
#endif
}

static bool connectWifiCredential(WifiCredential &c, uint32_t timeoutMs) {
  if (c.ssid.length() == 0) return false;

  Serial.print("WiFi connect: ");
  Serial.print(c.ssid);
  Serial.print(c.enterprise ? " (WPA2 Enterprise)" : " (WPA/WPA2 Personal)");
  if (c.channel > 0) {
    Serial.print(" ch=");
    Serial.print(c.channel);
  }
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);
  delay(80);

#if POTENV2_HAS_WPA2_ENTERPRISE
  esp_wifi_sta_wpa2_ent_disable();
#endif

  if (c.enterprise || c.user.length() > 0) {
    configureEnterpriseWifi(c);
    WiFi.begin(c.ssid.c_str());
  } else if (c.channel > 0 && c.hasBssid) {
    WiFi.begin(c.ssid.c_str(), c.pass.c_str(), c.channel, c.bssid, true);
  } else if (c.channel > 0) {
    WiFi.begin(c.ssid.c_str(), c.pass.c_str(), c.channel);
  } else {
    WiFi.begin(c.ssid.c_str(), c.pass.c_str());
  }

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  bool ok = (WiFi.status() == WL_CONNECTED);
  if (ok) {
    Serial.print("WiFi connected: ");
    Serial.print(WiFi.SSID());
    Serial.print(" IP=");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed.");
    WiFi.disconnect(false, false);
  }
  return ok;
}

static bool connectDefaultWifiFirst() {
  WifiCredential c;
  c.ssid = String(DEFAULT_WIFI_SSID);
  c.pass = String(DEFAULT_WIFI_PASSWORD);
  c.user = String(DEFAULT_WIFI_USER);
  c.enterprise = c.user.length() > 0;
  if (c.ssid.length() == 0) return false;
  Serial.println("Trying default WiFi first...");
  return connectWifiCredential(c, WIFI_DEFAULT_TIMEOUT_MS);
}

static bool connectStoredWifiFast() {
  loadWifiCredentials();
  if (wifiCredCount == 0) return false;

  Serial.println("Fast scan for stored SSIDs...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  int n = WiFi.scanNetworks(false, true, false, WIFI_SCAN_MS_PER_CH, 0);

  int bestCred = -1;
  int bestAp = -1;
  int bestRssi = -999;

  if (n > 0) {
    for (int ap = 0; ap < n; ap++) {
      String apSsid = WiFi.SSID(ap);
      for (uint8_t i = 0; i < wifiCredCount; i++) {
        if (wifiCreds[i].ssid == apSsid && WiFi.RSSI(ap) > bestRssi) {
          bestRssi = WiFi.RSSI(ap);
          bestCred = i;
          bestAp = ap;
        }
      }
    }
  }

  if (bestCred >= 0) {
    wifiCreds[bestCred].channel = WiFi.channel(bestAp);
    uint8_t *b = WiFi.BSSID(bestAp);
    if (b) {
      memcpy(wifiCreds[bestCred].bssid, b, 6);
      wifiCreds[bestCred].hasBssid = true;
    }
    wifiCreds[bestCred].lastRssi = bestRssi;

    WifiCredential selected = wifiCreds[bestCred];
    WiFi.scanDelete();
    if (connectWifiCredential(selected, WIFI_STORED_TIMEOUT_MS)) {
      // Move successful credential to top and save updated BSSID/channel.
      WifiCredential ok = selected;
      for (int i = bestCred; i > 0; i--) wifiCreds[i] = wifiCreds[i - 1];
      wifiCreds[0] = ok;
      saveWifiCredentials();
      return true;
    }
  } else {
    WiFi.scanDelete();
  }

  // Fallback: try stored credentials with known channel/BSSID first, short timeout each.
  for (uint8_t i = 0; i < wifiCredCount; i++) {
    if (connectWifiCredential(wifiCreds[i], WIFI_STORED_TIMEOUT_MS)) {
      WifiCredential ok = wifiCreds[i];
      for (int j = i; j > 0; j--) wifiCreds[j] = wifiCreds[j - 1];
      wifiCreds[0] = ok;
      saveWifiCredentials();
      return true;
    }
  }
  return false;
}

static String wifiStatusHtml() {
  String s;
  if (WiFi.status() == WL_CONNECTED) {
    s += F("<p class='ok'>Connected to <b>");
    s += htmlEscape(WiFi.SSID());
    s += F("</b><br>IP: ");
    s += WiFi.localIP().toString();
    s += F("</p>");
  } else {
    s += F("<p class='warn'>Not connected. AP portal is active.</p>");
    s += F("<p>AP SSID: <b>");
    s += htmlEscape(wifiApSsid);
    s += F("</b>, password: <b>");
    s += WIFI_AP_PASSWORD;
    s += F("</b>, open <b>192.168.4.1</b></p>");
  }
  return s;
}

static String wifiPortalPage() {
  loadWifiCredentials();
  String html = F(
    "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>smart-ec WiFi Manager</title>"
    "<style>body{font-family:Arial;margin:20px;max-width:760px}input,button{font-size:16px;padding:8px;margin:4px 0;width:100%;box-sizing:border-box}button{cursor:pointer}.card{border:1px solid #ccc;border-radius:10px;padding:12px;margin:12px 0}.ok{color:#087d16}.warn{color:#b06000}.row{display:grid;grid-template-columns:1fr 100px;gap:8px;align-items:center}.small{font-size:13px;color:#666}</style>"
    "</head><body><h2>smart-ec WiFi Manager</h2>");
  html += wifiStatusHtml();
  html += F(
    "<div class='card'><h3>Add / Update WiFi</h3>"
    "<form method='POST' action='/save'>"
    "<label>SSID</label><input name='ssid' maxlength='32' required>"
    "<label>Password</label><input name='pass' type='password' maxlength='64'>"
    "<label><input type='checkbox' name='enterprise' value='1' style='width:auto'> WPA2 Enterprise</label><br>"
    "<label>Enterprise Username / Identity</label><input name='user' maxlength='64' placeholder='Leave blank for normal WPA/WPA2'>"
    "<button type='submit'>Save and connect</button></form></div>"
    "<div class='card'><h3>Saved Networks</h3>");

  if (wifiCredCount == 0) {
    html += F("<p>No saved network.</p>");
  } else {
    for (uint8_t i = 0; i < wifiCredCount; i++) {
      html += F("<div class='row'><div><b>");
      html += htmlEscape(wifiCreds[i].ssid);
      html += F("</b><div class='small'>");
      html += wifiCreds[i].enterprise ? F("WPA2 Enterprise") : F("WPA/WPA2 Personal");
      if (wifiCreds[i].channel > 0) {
        html += F(" · ch ");
        html += String(wifiCreds[i].channel);
      }
      html += F("</div></div><a href='/delete?i=");
      html += String(i);
      html += F("'><button type='button'>Delete</button></a></div>");
    }
  }

  html += F(
    "</div><div class='card'><h3>Tools</h3>"
    "<a href='/scan'><button type='button'>Scan WiFi JSON</button></a>"
    "<a href='/connect'><button type='button'>Try saved networks now</button></a>"
    "<p class='small'>WiFi credentials are stored in NVS Preferences. The HTML portal is stored in LittleFS.</p>"
    "</div></body></html>");
  return html;
}


static void sendWifiPortalIndex() {
  if (LittleFS.exists(WIFI_PORTAL_FILE)) {
    File f = LittleFS.open(WIFI_PORTAL_FILE, FILE_READ);
    if (f) {
      wifiServer.streamFile(f, "text/html");
      f.close();
      return;
    }
  }

  // Fallback page, useful when firmware is updated but filesystem was not uploaded yet.
  String html = F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                  "<title>smart-ec WiFi</title></head><body style='font-family:Arial;margin:22px'>"
                  "<h2>smart-ec WiFi Manager</h2>"
                  "<p><b>wifi_index.html not found in LittleFS.</b></p>"
                  "<p>Upload the PlatformIO data folder to LittleFS, or use this simple form.</p>"
                  "<form method='POST' action='/save'>"
                  "SSID<br><input name='ssid' required maxlength='32'><br>"
                  "Password<br><input name='pass' type='password' maxlength='64'><br>"
                  "Enterprise user<br><input name='user' maxlength='64'><br>"
                  "<label><input type='checkbox' name='enterprise' value='1'> WPA2 Enterprise</label><br><br>"
                  "<button type='submit'>Save and connect</button></form>"
                  "<p><a href='/api/status'>Status JSON</a> | <a href='/scan'>Scan JSON</a></p>"
                  "</body></html>");
  wifiServer.send(200, "text/html", html);
}

static String wifiStatusJson() {
  String json = F("{");
  json += F("\"connected\":");
  json += (WiFi.status() == WL_CONNECTED) ? F("true") : F("false");
  json += F(",\"sta_ssid\":\"");
  json += jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.SSID() : String(""));
  json += F("\",\"ip\":\"");
  json += jsonEscape(WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String(""));
  json += F("\",\"ap_active\":");
  json += wifiApMode ? F("true") : F("false");
  json += F(",\"ap_ssid\":\"");
  json += jsonEscape(wifiApSsid);
  json += F("\",\"ap_password\":\"");
  json += jsonEscape(WIFI_AP_PASSWORD);
  json += F("\",\"credential_storage\":\"NVS Preferences\"");
  json += F(",\"nvs_namespace\":\"");
  json += jsonEscape(WIFI_NVS_NAMESPACE);
  json += F("\",\"nvs_key\":\"");
  json += jsonEscape(WIFI_NVS_KEY);
  json += F("\",\"portal_file\":\"");
  json += jsonEscape(WIFI_PORTAL_FILE);
  json += F("\",\"saved_count\":");
  json += String(wifiCredCount);
  json += F("}");
  return json;
}

static String wifiNetworksJson() {
  loadWifiCredentials();
  String json = F("{\"networks\":[");
  for (uint8_t i = 0; i < wifiCredCount; i++) {
    if (i) json += ',';
    WifiCredential &c = wifiCreds[i];
    json += F("{\"id\":");
    json += String(i);
    json += F(",\"ssid\":\"");
    json += jsonEscape(c.ssid);
    json += F("\",\"enterprise\":");
    json += c.enterprise ? F("true") : F("false");
    json += F(",\"user\":\"");
    json += jsonEscape(c.user);
    json += F("\",\"channel\":");
    json += String(c.channel);
    json += F(",\"has_bssid\":");
    json += c.hasBssid ? F("true") : F("false");
    json += F(",\"bssid\":\"");
    json += c.hasBssid ? bssidToString(c.bssid) : String("");
    json += F("\"}");
  }
  json += F("]}");
  return json;
}

static void handleWifiSave() {
  WifiCredential c;
  c.ssid = wifiServer.arg("ssid");
  c.pass = wifiServer.arg("pass");
  c.user = wifiServer.arg("user");
  c.ssid.trim();
  c.user.trim();
  c.enterprise = wifiServer.hasArg("enterprise") || wifiServer.arg("enterprise") == "1" || c.user.length() > 0;

  if (c.ssid.length() == 0) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"SSID is required\"}"));
    return;
  }

  upsertWifiCredential(c);
  wifiConnectRequested = true;
  wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Saved. Connecting...\"}"));
}

static void handleWifiDelete() {
  int idx = -1;
  if (wifiServer.hasArg("i")) idx = wifiServer.arg("i").toInt();
  else if (wifiServer.hasArg("id")) idx = wifiServer.arg("id").toInt();

  if (idx < 0 || idx >= wifiCredCount) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"Invalid network id\"}"));
    return;
  }

  deleteWifiCredential((uint8_t)idx);
  wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Deleted\"}"));
}

static String wifiScanJson() {
  int n = WiFi.scanNetworks(false, true, false, WIFI_SCAN_MS_PER_CH, 0);
  String json = F("{\"scan_result\":[");
  for (int i = 0; i < n; i++) {
    if (i) json += ',';
    json += F("{\"ssid\":\"");
    json += jsonEscape(WiFi.SSID(i));
    json += F("\",\"rssi\":");
    json += String(WiFi.RSSI(i));
    json += F(",\"channel\":");
    json += String(WiFi.channel(i));
    json += F(",\"encryption\":");
    json += String((int)WiFi.encryptionType(i));
    json += F("}");
  }
  json += F("]}");
  WiFi.scanDelete();
  return json;
}

static void startWifiPortal() {
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06X", (uint32_t)(mac & 0xFFFFFF));
  wifiApSsid = String("smart-ec-Setup-") + suffix;

  // If the web server routes were already registered earlier, only re-enable the AP radio.
  // This lets the setup portal come back after STA disconnects without registering routes twice.
  if (wifiServerStarted) {
    if (!wifiApMode) {
      WiFi.mode(WIFI_AP_STA);
      WiFi.setSleep(false);
      bool apOK = (strlen(WIFI_AP_PASSWORD) >= 8) ? WiFi.softAP(wifiApSsid.c_str(), WIFI_AP_PASSWORD) : WiFi.softAP(wifiApSsid.c_str());
      delay(100);
      IPAddress apIP = WiFi.softAPIP();
      wifiDns.start(53, "*", apIP);
      wifiApMode = true;
      Serial.print("WiFi AP portal restarted ");
      Serial.print(apOK ? "OK" : "FAILED");
      Serial.print(" SSID=");
      Serial.print(wifiApSsid);
      Serial.print(" IP=");
      Serial.println(apIP);
    }
    return;
  }

  wifiApMode = true;
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  bool apOK = (strlen(WIFI_AP_PASSWORD) >= 8) ? WiFi.softAP(wifiApSsid.c_str(), WIFI_AP_PASSWORD) : WiFi.softAP(wifiApSsid.c_str());
  delay(100);
  IPAddress apIP = WiFi.softAPIP();
  wifiDns.start(53, "*", apIP);

  wifiServer.on("/", HTTP_GET, []() { sendWifiPortalIndex(); });
  wifiServer.on("/wifi_index.html", HTTP_GET, []() { sendWifiPortalIndex(); });
  wifiServer.on("/generate_204", HTTP_GET, []() { wifiServer.sendHeader("Location", "/", true); wifiServer.send(302, "text/plain", ""); });
  wifiServer.on("/fwlink", HTTP_GET, []() { wifiServer.sendHeader("Location", "/", true); wifiServer.send(302, "text/plain", ""); });

  wifiServer.on("/api/status", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiStatusJson()); });
  wifiServer.on("/api/networks", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiNetworksJson()); });
  wifiServer.on("/api/scan", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiScanJson()); });
  wifiServer.on("/api/save", HTTP_POST, []() { handleWifiSave(); });
  wifiServer.on("/api/delete", HTTP_POST, []() { handleWifiDelete(); });
  wifiServer.on("/api/connect", HTTP_POST, []() {
    wifiConnectRequested = true;
    wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Connecting...\"}"));
  });

  // Backward-compatible routes for old forms/tools.
  wifiServer.on("/save", HTTP_POST, []() { handleWifiSave(); });
  wifiServer.on("/delete", HTTP_GET, []() { handleWifiDelete(); });
  wifiServer.on("/connect", HTTP_GET, []() {
    wifiConnectRequested = true;
    wifiServer.sendHeader("Location", "/", true);
    wifiServer.send(302, "text/plain", "");
  });
  wifiServer.on("/scan", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiScanJson()); });

  wifiServer.onNotFound([]() {
    wifiServer.sendHeader("Location", "/", true);
    wifiServer.send(302, "text/plain", "");
  });

  wifiServer.begin();
  wifiServerStarted = true;

  Serial.print("WiFi AP portal ");
  Serial.print(apOK ? "started" : "failed");
  Serial.print(" SSID=");
  Serial.print(wifiApSsid);
  Serial.print(" IP=");
  Serial.println(apIP);
}

static void stopWifiPortalIfConnected() {
  if (!wifiApMode || WiFi.status() != WL_CONNECTED) return;
  // Keep the web server object alive, but stop AP radio to reduce noise/current.
  wifiDns.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  wifiApMode = false;
  Serial.println("WiFi AP portal stopped after STA connected.");
}

static void wifiManagerBeginFromOptionTask() {
  Serial.println("WiFi manager: default first, then fast-scan NVS credentials, then AP portal.");
  if (connectDefaultWifiFirst()) return;
  if (connectStoredWifiFast()) return;
  startWifiPortal();
}

static void wifiManagerLoop() {
  if (wifiServerStarted) {
    wifiDns.processNextRequest();
    wifiServer.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED) {
    stopWifiPortalIfConnected();
    return;
  }

  uint32_t nowMs = millis();
  if (wifiConnectRequested || (!wifiApMode && nowMs - wifiLastRetryMs > WIFI_RETRY_PERIOD_MS)) {
    wifiConnectRequested = false;
    wifiLastRetryMs = nowMs;
    if (!connectDefaultWifiFirst() && !connectStoredWifiFast()) {
      startWifiPortal();
    }
  }
}



void optionTask(void *pvParameters) {
  Serial.println("OptionTask started on Core " + String(xPortGetCoreID()) + " (low priority, Serial/logger/options/WiFi manager)");
  Serial.println();
  Serial.println("direction,DAC_set_V,DAC_code,DAC_expected_V,ADC_raw,ADC_V,ADC_delta_V,current_A,RTIA_ohm,TIACN,AutoRange,timestamp_us,period_us");

  MeasurementData data;
  uint32_t lastI2CStatusMs = 0;

  wifiManagerBeginFromOptionTask();
  requestTftFullRefresh();

  while (true) {
    // 1) Serial commands
    while (Serial.available()) {
      handleSerialCommand((char)Serial.read());
    }

#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL && ENABLE_TOUCH_POLLING
    handleTouchControl();
#endif

    // 2) Logger: drain a limited number per loop to avoid CPU0 WDT / Serial blocking too long.
    uint8_t printed = 0;
    while (printed < 4 && xQueueReceive(loggerQueue, &data, 0) == pdTRUE) {
      printMeasurementCsv(data);
      printed++;
    }

#if ENABLE_I2C_MONITOR_TASK
    // 3) Optional slow I2C monitor. Keep disabled while debugging measurement.
    uint32_t nowMs = millis();
    if (!measurementEnabled && (nowMs - lastI2CStatusMs >= 1000)) {
      lastI2CStatusMs = nowMs;
      uint8_t status = lmpReadReg_safe(REG_STATUS);
      (void)status;
    }
#endif

#if ENABLE_TFT_DISPLAY
    serviceTftRefresh();
#endif

    wifiManagerLoop();

    // Always yield. OptionTask must never starve the idle task.
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ======================================================
// Setup
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("smart-ec Rev.2.0 Multitask Test");
  Serial.println("DAC AD5680 + ADC AD7694 + LMP91000 + TFT ILI9488");
  Serial.println();

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);

  // ------------------------------
  // FreeRTOS objects first
  // ------------------------------
  spiMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  loggerQueue = xQueueCreate(256, sizeof(MeasurementData));

  if (!spiMutex || !i2cMutex || !loggerQueue) {
    Serial.println("FreeRTOS object create failed.");
    while (true) delay(1000);
  }

#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed. Touch calibration and HTML portal will not be available.");
  } else {
    Serial.println("LittleFS mounted OK.");
  }
#endif

  // ------------------------------
  // GPIO init
  // ------------------------------
  pinMode(DAC_SYNC, OUTPUT);
  digitalWrite(DAC_SYNC, HIGH);

  pinMode(ADC_CNV, OUTPUT);
  digitalWrite(ADC_CNV, HIGH); // AD7694 deselect/tri-state MISO while sharing SPI

  if (LMP_MENB_PIN >= 0) {
    pinMode(LMP_MENB_PIN, OUTPUT);
    digitalWrite(LMP_MENB_PIN, LOW);
  }

  // ------------------------------
  // I2C init
  // ------------------------------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // ------------------------------
  // Shared Hardware SPI init for DAC/ADC
  // LovyanGFX also uses SPI2_HOST with the same pins.
  // ------------------------------
#if ENABLE_TFT_DISPLAY
  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

#if ENABLE_TOUCH_CONTROL
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  if (TOUCH_IRQ >= 0) {
    pinMode(TOUCH_IRQ, INPUT_PULLUP);
  }
#endif

  if (TFT_BL >= 0) {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }
#endif

  spiBus.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
  deselectAllSPI();

#if ENABLE_TFT_DISPLAY
  Serial.println("Init LovyanGFX TFT...");
  Serial.flush();

  if (takeSPI(1000)) {
    deselectAllSPI();
    lcd.init();
    lcd.setRotation(1);
#if ENABLE_TOUCH_CONTROL
    loadTouchCalibration();
#endif
    drawStaticScreen();
    drawRunButton(measurementEnabled);
    giveSPI();
    Serial.println("TFT init OK.");
  } else {
    Serial.println("TFT init failed: spiMutex timeout");
  }

#if ENABLE_TOUCH_CONTROL
  Serial.println("Touch control enabled: XPT2046 on shared SPI, CS=7, IRQ=15. Working-driver path.");
#else
  Serial.println("Touch control disabled.");
#endif
#endif
  // ------------------------------
  // LMP91000 config before measurement starts
  // ------------------------------
  Serial.println("Init LMP91000...");
  bool lmpOK = lmpInitFromYourCode();

  if (lmpOK) {
    Serial.println("LMP91000 init OK.");
  } else {
    Serial.println("LMP91000 init FAILED.");
  }

  printLMPRegisters_safe();
  Serial.print("Auto range: ");
  Serial.println(ENABLE_AUTO_RANGE ? "ENABLED" : "DISABLED");
  Serial.print("Initial TIA range: ");
  Serial.print(getCurrentRangeName());
  Serial.print(" RTIA=");
  Serial.println(getCurrentRtiaOhm(), 0);

  // ------------------------------
  // DAC initial
  // ------------------------------
  writeAD5680_safe(0);
  delay(500);

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  r = run measurement, reset RTIA to 2.75k first");
  Serial.println("  s = stop measurement");
  Serial.println("  p = print LMP91000 registers");
  Serial.println("  g = set TIA range back to 35k");
  Serial.println("  u = manual TFT refresh, STOP only");
  Serial.println("  t = touch status once");
  Serial.println("  w = raw XPT2046 touch values");
  Serial.println("  c = calibrate touch and save to LittleFS");
  Serial.println("  x = delete touch calibration file");
  Serial.println("  d = print diagnostics");
  Serial.println("  WiFi manager: default WiFi -> NVS saved networks -> AP portal");
  Serial.println("  Portal file: /wifi_index.html in LittleFS");
  Serial.println("  Credentials: NVS namespace smart_ec, key wifi_list_v1");
  Serial.println();


  // ------------------------------
  // Create tasks
  // ------------------------------
  xTaskCreatePinnedToCore(
    measurementTask,
    "MeasurementTask",
    8192,
    nullptr,
    MEAS_PRIORITY_IDLE,
    &measurementTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    optionTask,
    "OptionTask",
    12288,
    nullptr,
    OPTION_PRIORITY,
    &optionTaskHandle,
    0
  );

  Serial.println("Tasks created. Ready.");
}

// ======================================================
// Loop
// ======================================================
void loop() {
  // Nothing here. All work is done by FreeRTOS tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
