#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "esp_timer.h"

// ======================================================
// PotenV2 Rev.2.0 Multitasking Example
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
      cfg.freq_write  = 40000000;       // Same as working touch test file
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;

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
  deselectAllSPI();
  lcd.fillScreen(TFT_BLACK);
  lcd.setRotation(1);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(10, 10);
  lcd.println("PotenV2 Multitask");

  lcd.setTextSize(1);
  lcd.setCursor(10, 42);
  lcd.println("LovyanGFX + Shared HW SPI");
  lcd.setCursor(10, 58);
  lcd.println("Core1: Measurement  Core0: Option");

  // Start/Stop button area
  lcd.drawRect(BTN_X, BTN_Y, BTN_W, BTN_H, TFT_WHITE);
}

void drawRunButton(bool running) {
  deselectAllSPI();
  uint16_t bg = running ? TFT_RED : TFT_GREEN;
  lcd.fillRect(BTN_X + 2, BTN_Y + 2, BTN_W - 4, BTN_H - 4, bg);
  lcd.setTextColor(TFT_BLACK, bg);
  lcd.setTextSize(2);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString(running ? "STOP" : "RUN", BTN_X + BTN_W / 2, BTN_Y + BTN_H / 2);
  lcd.setTextDatum(TL_DATUM);
}

void drawMeasurement(const MeasurementData &data) {
  deselectAllSPI();
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
}
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

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    Serial.println("TFT refresh failed: spiMutex timeout");
    return;
  }

  drawRunButton(false);
  if (hasLastMeasurement) {
    drawMeasurement(lastMeasurement);
  }

  xSemaphoreGive(spiMutex);
  vTaskDelay(pdMS_TO_TICKS(10));
  Serial.println("TFT refreshed");
}
#else
void manualRefreshDisplay() {
  Serial.println("TFT disabled");
}
#endif

void refreshRunButtonFromOptionTask() {
#if ENABLE_TFT_DISPLAY
  if (!spiMutex) return;
  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
  drawRunButton(measurementEnabled);
  xSemaphoreGive(spiMutex);
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
  File f = SPIFFS.open(CALIBRATION_FILE, "w");
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
  if (!SPIFFS.exists(CALIBRATION_FILE)) {
    Serial.println("No touch calibration file found.");
    return false;
  }

  File f = SPIFFS.open(CALIBRATION_FILE, "r");
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
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    SPIFFS.remove(CALIBRATION_FILE);
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

void optionTask(void *pvParameters) {
  Serial.println("OptionTask started on Core " + String(xPortGetCoreID()) + " (low priority, Serial/logger/options)");
  Serial.println();
  Serial.println("direction,DAC_set_V,DAC_code,DAC_expected_V,ADC_raw,ADC_V,ADC_delta_V,current_A,RTIA_ohm,TIACN,AutoRange,timestamp_us,period_us");

  MeasurementData data;
  uint32_t lastI2CStatusMs = 0;

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
  Serial.println("PotenV2 Rev.2.0 Multitask Test");
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
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed. Touch calibration will not be saved.");
  } else {
    Serial.println("SPIFFS mounted OK.");
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
  Serial.println("  c = calibrate touch and save to SPIFFS");
  Serial.println("  x = delete touch calibration file");
  Serial.println("  d = print diagnostics");
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
    8192,
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
