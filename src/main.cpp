#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FS.h>
#include <LittleFS.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <time.h>
#include "esp_system.h"
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
#define ENABLE_SD_LOGGING        1

static const char *FW_VERSION  = "2.0.0";
static const char *WEB_VERSION = "2.0.0";

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
#define RUN_LOGO_FILE "/run_logo.png"
#define BRAND_LOGO_FILE "/brand_logo.png"

// LMP91000 MENB
// If MENB is not connected, use -1.
#define LMP_MENB_PIN -1

// Optional built-in LED
#define LED_BUILTIN_PIN 2

// SD_MMC, same pin mapping as backup/SDmmc.cpp.
#define SD_CLK 39
#define SD_CMD 38
#define SD_D0  40
#define SD_D1  41
#define SD_D2  42
#define SD_D3  18

// Vibration motor control (optional)
#define VIB_MOTOR_PIN 6

#define VIB_PWM_CH   0
#define VIB_PWM_FREQ 250     // Hz, เหมาะกับ vibration motor
#define VIB_PWM_RES  8       // 8-bit = 0-255

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
static const uint32_t PULSE_TAIL_AVG_US   = 20000;   // SWV/DPV: average the stable tail of each pulse
static const uint32_t PULSE_TAIL_AVG_MIN_US = 1000;  // For very short pulses, use all remaining hold time
static const uint32_t SWEEP_INTERVAL_MS   = 3000;
static const uint32_t HOLD_CENTER_INTERVAL_MS = 250;
static const uint8_t  SD_WRITE_BATCH_LIMIT = 32;
static const uint32_t SD_FLUSH_INTERVAL_MS = 5000;
static const uint16_t CV_SERIAL_LOG_EVERY = 200;
static const float POTENTIAL_ZERO_V = ADC_VREF * 0.5f; // E=0V maps to DAC/RE 2.048V
static const float CV_MIN_POTENTIAL_V = -POTENTIAL_ZERO_V;
static const float CV_MAX_POTENTIAL_V = DAC_VREF - POTENTIAL_ZERO_V;
static const float CONTACT_PRECHECK_PULSE_V = 0.020f;
static const uint32_t CONTACT_PRECHECK_HOLD_MS = 250;
static const uint8_t CONTACT_PRECHECK_ADC_SAMPLES = 8;
static const float CONTACT_PRECHECK_RAIL_MARGIN_V = 0.100f;
static const float CONTACT_PRECHECK_MIN_RESPONSE_V = 0.002f;
static const float CONTACT_PRECHECK_MIN_CURRENT_A = 50.0e-9f;

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

static const char *resetReasonToText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

// ======================================================
// Shared Hardware SPI + LovyanGFX
// ======================================================
// ใช้ Hardware SPI bus เดียวกันสำหรับ TFT, DAC, ADC
// สำคัญ: ทุก transaction ต้องถือ spiMutex และ deselect chip อื่นก่อนเสมอ
SPIClass spiBus(FSPI);   // ESP32-S3: FSPI usually maps to SPI2_HOST / SPI2_HOST in LovyanGFX

// เริ่ม conservative ก่อน ถ้านิ่งแล้วค่อยเพิ่มความเร็ว
SPISettings dacSPI(5000000, MSBFIRST, SPI_MODE1);  // AD5680: ถ้า DAC ไม่ถูกลอง SPI_MODE0
SPISettings adcSPI(5000000, MSBFIRST, SPI_MODE0);  // AD7694: เริ่ม 5 MHz ก่อน

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
      cfg.freq_write  = 20000000;       // Fast config-screen refresh; measurement disables TFT/Touch
      cfg.freq_read   = 8000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = 1;              // Enable DMA for TFT drawing while outside measurement sessions

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
      cfg.readable         = false; // ไม่ต้องอ่านข้อมูลกลับจากจอ
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

// PCF85363A RTC, UTC time base.
#define PCF85363_ADDR 0x51
#define PCF_REG_100TH_SECONDS 0x00
#define PCF_REG_SECONDS       0x01
#define PCF_REG_MINUTES       0x02
#define PCF_REG_STOP_ENABLE   0x2E
#define PCF_REG_RESETS        0x2F

// ======================================================
// FreeRTOS objects
// ======================================================
static SemaphoreHandle_t spiMutex = nullptr;
static SemaphoreHandle_t i2cMutex = nullptr;

static TaskHandle_t measurementTaskHandle = nullptr;
static TaskHandle_t optionTaskHandle = nullptr;
static TaskHandle_t sdWriterTaskHandle = nullptr;

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
static void resetMeasurementProgress(uint16_t totalCycles);
static void showTftRunStatusImmediate();

static QueueHandle_t loggerQueue = nullptr;
static QueueHandle_t sdRecordQueue = nullptr;

static bool initSdCard();
static String wifiMdnsUrl();
static bool wifiPortalApActive();
static String wifiPortalApSsid();
void vibrationOn(uint8_t duty);
void vibrationOff();
void vibrationPulse(uint8_t duty, uint32_t durationMs);

// ======================================================
// Runtime state
// ======================================================
volatile bool measurementEnabled = false;
volatile bool measurementBusy = false;
volatile bool measurementStopRequested = true;
volatile bool measurementPriorityIsHigh = false;
volatile bool serialDebugMode = false;
volatile bool measurementCompleted = false;
volatile uint16_t measurementCycleCurrent = 0;
volatile uint16_t measurementCycleTotal = 0;
volatile uint16_t measurementProgressPermille = 0;
static bool serialCsvHeaderPrinted = false;
static bool serialDebugModeLastReported = false;
static uint32_t lastSerialInputMs = 0;

enum TftRunStatus : uint8_t {
  TFT_RUN_READY = 0,
  TFT_RUN_STARTING,
  TFT_RUN_COMPLETED,
  TFT_RUN_STOPPED
};

volatile TftRunStatus tftRunStatus = TFT_RUN_READY;

static bool serialConnectionPresent() {
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  return (bool)Serial;
#else
  // With an external USB-UART bridge there is no reliable passive connection
  // signal. Seeing RX data means a terminal/operator is present.
  return lastSerialInputMs != 0;
#endif
}

static void updateSerialDebugMode(bool serialInputSeen = false) {
  if (serialInputSeen) {
    lastSerialInputMs = millis();
  }

  bool connected = serialConnectionPresent();
  serialDebugMode = connected;

  if (serialDebugMode && !serialDebugModeLastReported) {
    Serial.println();
    Serial.println("Serial debug mode ON");
    Serial.println("mode,direction,DAC_set_V,DAC_code,DAC_expected_V,ADC_raw,ADC_V,ADC_delta_V,current_A,RTIA_ohm,TIACN,AutoRange,timestamp_us,period_us");
    serialCsvHeaderPrinted = true;
  }

  serialDebugModeLastReported = serialDebugMode;
}

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
  tftRunStatus = TFT_RUN_STARTING;
  showTftRunStatusImmediate();

  measurementStopRequested = false;
  resetMeasurementProgress(0);
  measurementEnabled = true;

  setMeasurementPriority(true);

  if (measurementTaskHandle != nullptr) {
    xTaskNotifyGive(measurementTaskHandle);
  }
}

void requestMeasurementStop() {
  // Do not forcibly suspend the task. Let it leave the measurement loop
  // at a safe point after the current ADC/DAC transaction.
  tftRunStatus = TFT_RUN_STOPPED;
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

enum MeasurementMode : uint8_t {
  MEAS_MODE_SWEEP = 0,
  MEAS_MODE_HOLD_CENTER = 1,
  MEAS_MODE_CV = 2,
  MEAS_MODE_SWV = 3,
  MEAS_MODE_DPV = 4
};

volatile MeasurementMode currentMeasurementMode = MEAS_MODE_SWEEP;

struct MeasurementData {
  MeasurementMode mode;
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

struct CvConfig {
  char experimentName[32];
  float startV;
  float vertex1V;
  float vertex2V;
  float finalV;
  float stepV;
  float scanRateVps;
  uint16_t cycles;
  uint32_t quietMs;
};

static CvConfig cvConfig = {
  "",       // experimentName
  0.0f,     // startV, relative to POTENTIAL_ZERO_V
  0.8f,     // vertex1V
  -0.8f,    // vertex2V
  0.0f,     // finalV
  0.01f,    // stepV
  0.1f,     // scanRateVps
  1,        // cycles
  0         // quietMs
};

struct SwvConfig {
  char experimentName[32];
  float startV;
  float endV;
  float stepV;
  float amplitudeV;
  float frequencyHz;
  float periodMs;
  float dutyMs;
  float dutyPercent;
  bool useFrequency;
  bool useDutyPercent;
  bool conditioningEnabled;
  float conditioningV;
  uint32_t conditioningMs;
  bool agitationEnabled;
  uint32_t agitationOnMs;
  uint32_t agitationOffMs;
  uint8_t motorPowerPercent;
  uint32_t settleAfterMs;
  uint32_t quietMs;
};

static SwvConfig swvConfig = {
  "",       // experimentName
  -0.5f,    // startV, relative to POTENTIAL_ZERO_V
  0.5f,     // endV
  0.005f,   // stepV
  0.025f,   // amplitudeV
  25.0f,    // frequencyHz
  40.0f,    // periodMs
  20.0f,    // dutyMs
  50.0f,    // dutyPercent
  false,    // useFrequency; default is period time input
  false,    // useDutyPercent; default is duty time input
  false,    // conditioningEnabled
  0.0f,     // conditioningV
  60000,    // conditioningMs
  false,    // agitationEnabled
  2000,     // agitationOnMs
  3000,     // agitationOffMs
  60,       // motorPowerPercent
  2000,     // settleAfterMs
  0         // quietMs
};

struct DpvConfig {
  char experimentName[32];
  float startV;
  float endV;
  float stepV;
  float amplitudeV;
  float periodMs;
  float pulseMs;
  bool conditioningEnabled;
  float conditioningV;
  uint32_t conditioningMs;
  bool agitationEnabled;
  uint32_t agitationOnMs;
  uint32_t agitationOffMs;
  uint8_t motorPowerPercent;
  uint32_t settleAfterMs;
  uint32_t quietMs;
};

static DpvConfig dpvConfig = {
  "",       // experimentName
  -0.5f,    // startV, relative to POTENTIAL_ZERO_V
  0.5f,     // endV
  0.005f,   // stepV
  0.050f,   // amplitudeV
  500.0f,   // periodMs
  50.0f,    // pulseMs
  false,    // conditioningEnabled
  0.0f,     // conditioningV
  60000,    // conditioningMs
  false,    // agitationEnabled
  2000,     // agitationOnMs
  3000,     // agitationOffMs
  60,       // motorPowerPercent
  2000,     // settleAfterMs
  0         // quietMs
};

struct CvBinHeader {
  char magic[8];
  uint16_t version;
  uint16_t headerSize;
  uint32_t recordSize;
  uint64_t startedUs;
  float zeroDacV;
  float dacVref;
  float adcVref;
  float startV;
  float vertex1V;
  float vertex2V;
  float finalV;
  float stepV;
  float scanRateVps;
  uint16_t cycles;
  uint16_t reserved;
  uint32_t quietMs;
  char experimentName[32];
};

struct CvBinRecord {
  uint32_t sampleIndex;
  int8_t direction;
  uint8_t autoRangeAction;
  uint8_t tiaCN;
  uint8_t reserved;
  uint64_t timestampUs;
  uint32_t periodUs;
  float potentialV;
  float dacSetV;
  float dacExpectedV;
  uint32_t dacCode;
  uint16_t adcRaw;
  uint16_t reserved2;
  float adcV;
  float adcDeltaV;
  float currentA;
  float rtiaOhm;
};

static_assert(sizeof(CvBinHeader) == 104, "Unexpected CV header size");
static_assert(sizeof(CvBinRecord) == 56, "Unexpected CV record size");

static CvBinHeader activeCvHeader;
static char activeCvFilePath[48] = "/cv_pending.bin";
volatile bool cvLogActive = false;
volatile bool sdMounted = false;
volatile bool sdLogFileOpen = false;
volatile uint32_t sdRecordDropCount = 0;
volatile uint32_t sdRecordWriteCount = 0;
volatile uint32_t sdWriteErrorCount = 0;
volatile bool rtcAvailable = false;
volatile bool rtcUtcSynced = false;
static uint32_t lastNtpSyncAttemptMs = 0;

struct RtcDateTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  bool oscillatorStopped;
};

static bool rtcReadUtc_safe(RtcDateTime &dt);

// Latest measurement for manual TFT refresh.
static MeasurementData lastMeasurement;
volatile bool hasLastMeasurement = false;

// Centralized TFT refresh requests.
// Only optionTask executes the actual lcd.* drawing.
// Touch/Serial/Web callbacks should request a refresh instead of drawing directly.
volatile bool tftFullRefreshRequested = false;
volatile bool tftStatusRefreshRequested = false;
volatile bool tftTimeRefreshRequested = false;
volatile bool tftRunButtonRefreshRequested = false;
volatile bool tftDrawMeasurementRequested = false;

static String formatBytesMb(uint64_t bytes) {
  return String((uint32_t)(bytes / (1024ULL * 1024ULL))) + " MB";
}

static bool formatRtcUtcDateTime(String &out) {
  RtcDateTime rtc;
  if (!rtcReadUtc_safe(rtc) || rtc.oscillatorStopped) return false;

  char buf[24];
  snprintf(
    buf,
    sizeof(buf),
    "%04u-%02u-%02u %02u:%02u",
    rtc.year,
    rtc.month,
    rtc.day,
    rtc.hour,
    rtc.minute
  );
  out = buf;
  return true;
}

struct TftStatusText {
  String wifiLine1;
  String wifiLine2;
  String wifiLine3;
  String sdLine;
  String rtcLine;
  String rtcTimeLine;
  String stateLine;
};

static String buildRtcTimeLineText() {
  String rtcText;
  if (!formatRtcUtcDateTime(rtcText)) rtcText = "--";
  return String("RTC UTC: ") + rtcText;
}

static TftStatusText buildTftStatusText() {
  TftStatusText text;

  if (WiFi.status() == WL_CONNECTED) {
    text.wifiLine1 = String("WiFi STA: ") + WiFi.SSID();
    text.wifiLine2 = String("Access: http://") + WiFi.localIP().toString();
    text.wifiLine3 = String("Local: ") + wifiMdnsUrl();
  } else if (wifiPortalApActive()) {
    text.wifiLine1 = "WiFi STA: not connected";
    text.wifiLine2 = String("Setup AP: ") + wifiPortalApSsid();
    text.wifiLine3 = "Access: http://192.168.4.1";
  } else {
    text.wifiLine1 = "WiFi STA: not connected";
    text.wifiLine2 = "";
    text.wifiLine3 = "";
  }

  if (sdMounted) {
    uint64_t totalBytes = SD_MMC.totalBytes();
    uint64_t usedBytes = SD_MMC.usedBytes();
    uint64_t freeBytes = (totalBytes > usedBytes) ? (totalBytes - usedBytes) : 0;
    text.sdLine = String("SD: free ") + formatBytesMb(freeBytes) + " / " + formatBytesMb(totalBytes);
  } else {
    text.sdLine = "SD: not mounted";
  }

  if (!rtcAvailable) {
    text.rtcLine = "RTC: not found";
  } else {
    text.rtcLine = String("RTC: ") + (rtcUtcSynced ? "UTC synced" : "not synced");
  }
  text.rtcTimeLine = buildRtcTimeLineText();
  text.stateLine = String("State: ") + (measurementEnabled ? "MEASURING - GLCD locked" : "IDLE");
  return text;
}

static bool measurementSessionActive() {
  return measurementEnabled || measurementBusy;
}

static void resetMeasurementProgress(uint16_t totalCycles) {
  measurementCompleted = false;
  measurementCycleCurrent = 0;
  measurementCycleTotal = totalCycles;
  measurementProgressPermille = 0;
}

static void setMeasurementProgress(uint32_t doneSteps, uint32_t totalSteps) {
  if (totalSteps == 0) {
    measurementProgressPermille = 0;
    return;
  }
  if (doneSteps > totalSteps) doneSteps = totalSteps;
  measurementProgressPermille = (uint16_t)((doneSteps * 1000UL) / totalSteps);
}

void requestTftFullRefresh() {
  tftFullRefreshRequested = true;
  tftStatusRefreshRequested = true;
  tftRunButtonRefreshRequested = true;
  tftDrawMeasurementRequested = false;
}

void requestTftStatusRefresh() {
  tftStatusRefreshRequested = true;
}

void requestTftTimeRefresh() {
  tftTimeRefreshRequested = true;
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

static const char *measurementModeToText(MeasurementMode mode) {
  if (mode == MEAS_MODE_DPV) return "DPV";
  if (mode == MEAS_MODE_SWV) return "SWV";
  if (mode == MEAS_MODE_CV) return "CV";
  if (mode == MEAS_MODE_HOLD_CENTER) return "HOLD_CENTER";
  return "SWEEP";
}

static MeasurementMode parseMeasurementMode(const String &value, MeasurementMode fallback) {
  String mode = value;
  mode.trim();
  mode.toLowerCase();

  if (mode == "sweep" || mode == "0") return MEAS_MODE_SWEEP;
  if (mode == "potentiostat_cv" || mode == "cv_mode" || mode == "cv" || mode == "3") return MEAS_MODE_CV;
  if (mode == "swv" || mode == "square_wave" || mode == "square_wave_voltammetry" || mode == "4") return MEAS_MODE_SWV;
  if (mode == "dpv" || mode == "differential_pulse" || mode == "differential_pulse_voltammetry" || mode == "5") return MEAS_MODE_DPV;
  if (mode == "hold" || mode == "hold_center" || mode == "center" || mode == "1") return MEAS_MODE_HOLD_CENTER;
  return fallback;
}

static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static float potentialToDacVoltage(float potentialV) {
  return clampFloat(POTENTIAL_ZERO_V - potentialV, 0.0f, DAC_VREF);
}

static float dacVoltageToPotential(float dacVoltage) {
  return POTENTIAL_ZERO_V - dacVoltage;
}

static uint8_t decToBcd(uint8_t value) {
  return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static uint8_t bcdToDec(uint8_t value) {
  return (uint8_t)(((value >> 4) * 10) + (value & 0x0F));
}

static MeasurementMode getMeasurementMode() {
  return currentMeasurementMode;
}

static bool setMeasurementMode(MeasurementMode mode) {
  if (measurementEnabled || measurementBusy) {
    return false;
  }

  currentMeasurementMode = mode;
  resetMeasurementProgress(0);
  return true;
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

static bool rtcWriteReg_safe(uint8_t reg, uint8_t value) {
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(PCF85363_ADDR);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return err == 0;
}

static bool rtcReadRegs_safe(uint8_t startReg, uint8_t *buffer, uint8_t len) {
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);

  Wire.beginTransmission(PCF85363_ADDR);
  Wire.write(startReg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return false;
  }

  uint8_t received = Wire.requestFrom(PCF85363_ADDR, len);
  if (received != len) {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return false;
  }

  for (uint8_t i = 0; i < len; i++) {
    buffer[i] = Wire.read();
  }

  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return true;
}

static bool rtcProbe_safe() {
  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(PCF85363_ADDR);
  uint8_t err = Wire.endTransmission();
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  rtcAvailable = (err == 0);
  return rtcAvailable;
}

static bool rtcReadUtc_safe(RtcDateTime &dt) {
  uint8_t data[8];
  if (!rtcReadRegs_safe(PCF_REG_100TH_SECONDS, data, sizeof(data))) {
    rtcAvailable = false;
    return false;
  }

  dt.second = bcdToDec(data[1] & 0x7F);
  dt.minute = bcdToDec(data[2] & 0x7F);
  dt.hour = bcdToDec(data[3] & 0x3F);
  dt.day = bcdToDec(data[4] & 0x3F);
  dt.month = bcdToDec(data[6] & 0x1F);
  dt.year = 2000 + bcdToDec(data[7]);
  dt.oscillatorStopped = (data[1] & 0x80) != 0;

  rtcAvailable = true;
  return dt.year >= 2024 && dt.month >= 1 && dt.month <= 12 && dt.day >= 1 && dt.day <= 31;
}

static bool rtcSetUtcFromTm_safe(const tm &utc) {
  if (!rtcAvailable && !rtcProbe_safe()) return false;

  rtcWriteReg_safe(PCF_REG_STOP_ENABLE, 0x01);
  rtcWriteReg_safe(PCF_REG_RESETS, 0xA4);

  if (i2cMutex) xSemaphoreTake(i2cMutex, portMAX_DELAY);
  Wire.beginTransmission(PCF85363_ADDR);
  Wire.write(PCF_REG_100TH_SECONDS);
  Wire.write(0x00);
  Wire.write(decToBcd((uint8_t)utc.tm_sec));
  Wire.write(decToBcd((uint8_t)utc.tm_min));
  Wire.write(decToBcd((uint8_t)utc.tm_hour));
  Wire.write(decToBcd((uint8_t)utc.tm_mday));
  Wire.write(decToBcd((uint8_t)utc.tm_wday));
  Wire.write(decToBcd((uint8_t)(utc.tm_mon + 1)));
  Wire.write(decToBcd((uint8_t)((utc.tm_year + 1900) - 2000)));
  uint8_t err = Wire.endTransmission();
  if (i2cMutex) xSemaphoreGive(i2cMutex);

  rtcWriteReg_safe(PCF_REG_STOP_ENABLE, 0x00);

  rtcAvailable = (err == 0);
  rtcUtcSynced = rtcAvailable;
  return rtcAvailable;
}

static bool syncUtcTimeFromNtpToRtc(uint32_t timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) return false;
  if (!rtcAvailable && !rtcProbe_safe()) return false;

  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
  uint32_t startMs = millis();
  tm utc;
  memset(&utc, 0, sizeof(utc));

  while (millis() - startMs < timeoutMs) {
    if (getLocalTime(&utc, 250) && (utc.tm_year + 1900) >= 2024) {
      bool ok = rtcSetUtcFromTm_safe(utc);
      Serial.print("RTC UTC sync from NTP ");
      Serial.println(ok ? "OK" : "FAILED");
      requestTftStatusRefresh();
      return ok;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("RTC UTC sync timeout");
  return false;
}

static void maybeSyncUtcAfterWifiConnected() {
  if (rtcUtcSynced || WiFi.status() != WL_CONNECTED) return;
  if (measurementEnabled || measurementBusy) return;
  uint32_t nowMs = millis();
  if (nowMs - lastNtpSyncAttemptMs < 60000UL) return;
  lastNtpSyncAttemptMs = nowMs;
  syncUtcTimeFromNtpToRtc(5000);
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

static void makeCvHeader(CvBinHeader &header, const CvConfig &cfg, uint64_t startedUs) {
  memset(&header, 0, sizeof(header));
  memcpy(header.magic, "POTCVB1", 7);
  header.version = 1;
  header.headerSize = sizeof(CvBinHeader);
  header.recordSize = sizeof(CvBinRecord);
  header.startedUs = startedUs;
  header.zeroDacV = POTENTIAL_ZERO_V;
  header.dacVref = DAC_VREF;
  header.adcVref = ADC_VREF;
  header.startV = cfg.startV;
  header.vertex1V = cfg.vertex1V;
  header.vertex2V = cfg.vertex2V;
  header.finalV = cfg.finalV;
  header.stepV = cfg.stepV;
  header.scanRateVps = cfg.scanRateVps;
  header.cycles = cfg.cycles;
  header.quietMs = cfg.quietMs;
  strncpy(header.experimentName, cfg.experimentName, sizeof(header.experimentName) - 1);
}

static void makeSwvHeader(CvBinHeader &header, const SwvConfig &cfg, uint64_t startedUs) {
  float frequencyHz = cfg.useFrequency ? cfg.frequencyHz : (1000.0f / cfg.periodMs);
  float periodMs = 1000.0f / frequencyHz;
  float dutyMs = cfg.useDutyPercent ? (periodMs * cfg.dutyPercent * 0.01f) : cfg.dutyMs;
  dutyMs = clampFloat(dutyMs, 0.1f, periodMs - 0.1f);
  float dutyPercent = (periodMs > 0.0f) ? ((dutyMs / periodMs) * 100.0f) : 50.0f;

  memset(&header, 0, sizeof(header));
  memcpy(header.magic, "POTSWV1", 7);
  header.version = 1;
  header.headerSize = sizeof(CvBinHeader);
  header.recordSize = sizeof(CvBinRecord);
  header.startedUs = startedUs;
  header.zeroDacV = POTENTIAL_ZERO_V;
  header.dacVref = DAC_VREF;
  header.adcVref = ADC_VREF;
  header.startV = cfg.startV;
  header.vertex1V = cfg.endV;
  header.vertex2V = cfg.amplitudeV;
  header.finalV = dutyPercent;
  header.stepV = cfg.stepV;
  header.scanRateVps = frequencyHz;
  header.cycles = 1;
  header.quietMs = cfg.quietMs;
  strncpy(header.experimentName, cfg.experimentName, sizeof(header.experimentName) - 1);
}

static void makeDpvHeader(CvBinHeader &header, const DpvConfig &cfg, uint64_t startedUs) {
  memset(&header, 0, sizeof(header));
  memcpy(header.magic, "POTDPV1", 7);
  header.version = 1;
  header.headerSize = sizeof(CvBinHeader);
  header.recordSize = sizeof(CvBinRecord);
  header.startedUs = startedUs;
  header.zeroDacV = POTENTIAL_ZERO_V;
  header.dacVref = DAC_VREF;
  header.adcVref = ADC_VREF;
  header.startV = cfg.startV;
  header.vertex1V = cfg.endV;
  header.vertex2V = cfg.amplitudeV;
  header.finalV = cfg.pulseMs;
  header.stepV = cfg.stepV;
  header.scanRateVps = cfg.periodMs;
  header.cycles = 1;
  header.quietMs = cfg.quietMs;
  strncpy(header.experimentName, cfg.experimentName, sizeof(header.experimentName) - 1);
}

static void sanitizeExperimentName(const String &input, char *output, size_t outputSize) {
  if (outputSize == 0) return;
  size_t pos = 0;
  String trimmed = input;
  trimmed.trim();

  for (size_t i = 0; i < trimmed.length() && pos + 1 < outputSize; i++) {
    char c = trimmed[i];
    bool ok = (c >= 'a' && c <= 'z') ||
              (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') ||
              c == '_' ||
              c == '-';
    if (ok) {
      output[pos++] = c;
    } else if (c == ' ' || c == '.') {
      output[pos++] = '_';
    }
  }

  output[pos] = '\0';
}

static void makeMeasurementFilePath(const char *prefix, char *path, size_t pathSize, uint64_t startedUs, const char *experimentName) {
  RtcDateTime rtc;
  if (!prefix || prefix[0] == '\0') prefix = "cv";
  if (rtcReadUtc_safe(rtc) && !rtc.oscillatorStopped) {
    if (experimentName && experimentName[0] != '\0') {
      snprintf(
        path,
        pathSize,
        "/%s_%04u%02u%02u_%s.bin",
        prefix,
        rtc.year,
        rtc.month,
        rtc.day,
        experimentName
      );
    } else {
      snprintf(
        path,
        pathSize,
        "/%s_%04u%02u%02u_%02u%02u%02u.bin",
        prefix,
        rtc.year,
        rtc.month,
        rtc.day,
        rtc.hour,
        rtc.minute,
        rtc.second
      );
    }
    return;
  }

  if (experimentName && experimentName[0] != '\0') {
    snprintf(path, pathSize, "/%s_%llu_%s.bin", prefix, (unsigned long long)(startedUs / 1000ULL), experimentName);
  } else {
    snprintf(path, pathSize, "/%s_%llu.bin", prefix, (unsigned long long)(startedUs / 1000ULL));
  }
}

static void makeCvFilePath(char *path, size_t pathSize, uint64_t startedUs, const char *experimentName) {
  makeMeasurementFilePath("cv", path, pathSize, startedUs, experimentName);
}

static void queueCvRecord(const MeasurementData &data) {
#if ENABLE_SD_LOGGING
  if (!cvLogActive || !sdRecordQueue) return;
  if (data.mode != MEAS_MODE_CV && data.mode != MEAS_MODE_SWV && data.mode != MEAS_MODE_DPV) return;

  CvBinRecord record;
  memset(&record, 0, sizeof(record));
  record.sampleIndex = data.sampleIndex;
  record.direction = (int8_t)data.direction;
  record.autoRangeAction = (uint8_t)data.autoRangeAction;
  record.tiaCN = data.tiaCN;
  record.timestampUs = data.timestamp_us;
  record.periodUs = data.period_us;
  record.potentialV = dacVoltageToPotential(data.dacSetVoltage);
  record.dacSetV = data.dacSetVoltage;
  record.dacExpectedV = data.dacExpectedVoltage;
  record.dacCode = data.dacCode;
  record.adcRaw = data.adcRaw;
  record.adcV = data.adcVoltage;
  record.adcDeltaV = data.adcDeltaVoltage;
  record.currentA = data.currentAmp;
  record.rtiaOhm = data.rtiaOhm;

  if (xQueueSend(sdRecordQueue, &record, 0) != pdTRUE) {
    sdRecordDropCount++;
  }
#else
  (void)data;
#endif
}

// ======================================================
// Push measurement to queues
// ======================================================
void publishMeasurement(const MeasurementData &data) {
  // Keep latest value for manual display refresh.
  lastMeasurement = data;
  hasLastMeasurement = true;

  bool shouldQueueSerialLog = true;
  if (data.mode == MEAS_MODE_CV) {
    shouldQueueSerialLog = (CV_SERIAL_LOG_EVERY > 0) &&
                           ((data.sampleIndex % CV_SERIAL_LOG_EVERY) == 0);
  }
  shouldQueueSerialLog = shouldQueueSerialLog && serialDebugMode;

  // Logger queue is for human diagnostics. CV still records every sample to SD,
  // but Serial is rate-limited so it cannot steal time from long measurements.
  if (shouldQueueSerialLog && loggerQueue) {
    if (xQueueSend(loggerQueue, &data, 0) != pdTRUE) {
      loggerDropCount++;
    }
  }

  queueCvRecord(data);
}

// ======================================================
// Single measurement point
// ======================================================
uint32_t measureOnePoint(MeasurementMode mode, float dacSetVoltage, SweepDirection direction, uint32_t &sampleIndex) {
  MeasurementData data;

  uint64_t t0 = esp_timer_get_time();

  measurementBusy = true;

  data.mode = mode;
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
  return data.period_us;
}

uint32_t measureAdcAtCurrentDac(MeasurementMode mode, float dacSetVoltage, SweepDirection direction, uint32_t &sampleIndex) {
  MeasurementData data;

  uint64_t t0 = esp_timer_get_time();

  measurementBusy = true;

  data.mode = mode;
  data.sampleIndex = sampleIndex++;
  data.direction = direction;
  data.dacSetVoltage = dacSetVoltage;
  data.dacCode = voltageToDACCode(dacSetVoltage);
  data.dacExpectedVoltage = dacCodeToVoltage(data.dacCode);
  data.autoRangeAction = '-';

  data.adcRaw = readAD7694Average_safe(ADC_AVG_SAMPLES);
  data.adcVoltage = adcRawToVoltage(data.adcRaw);

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
  return data.period_us;
}

static void runSweepMode(uint32_t &sampleIndex) {
  Serial.println("Measurement sweep start");

  // Sweep up: 0.000V to 4.090V, step 10mV
  for (int mv = 0; mv <= 4090; mv += 10) {
    if (!measurementEnabled || measurementStopRequested) break;
    measureOnePoint(MEAS_MODE_SWEEP, mv / 1000.0f, DIR_UP, sampleIndex);
    vTaskDelay(1);
  }

  // Full-scale point 4.096V
  if (measurementEnabled && !measurementStopRequested) {
    measureOnePoint(MEAS_MODE_SWEEP, 4.096f, DIR_UP, sampleIndex);
    vTaskDelay(1);
  }

  // Sweep down: 4.090V to 0.000V, step 10mV
  for (int mv = 4090; mv >= 0; mv -= 10) {
    if (!measurementEnabled || measurementStopRequested) break;
    measureOnePoint(MEAS_MODE_SWEEP, mv / 1000.0f, DIR_DOWN, sampleIndex);
    vTaskDelay(1);
  }

  // Center point
  if (measurementEnabled && !measurementStopRequested) {
    measureOnePoint(MEAS_MODE_SWEEP, 2.048f, DIR_CENTER, sampleIndex);
  }

  Serial.println("Measurement sweep done");

  setMeasurementPriority(false);

  uint32_t waitedMs = 0;
  while (measurementEnabled && !measurementStopRequested && waitedMs < SWEEP_INTERVAL_MS) {
    vTaskDelay(pdMS_TO_TICKS(20));
    waitedMs += 20;
  }
}

static void runHoldCenterMode(uint32_t &sampleIndex) {
  measureOnePoint(MEAS_MODE_HOLD_CENTER, 2.048f, DIR_CENTER, sampleIndex);

  setMeasurementPriority(false);

  uint32_t waitedMs = 0;
  while (measurementEnabled && !measurementStopRequested && waitedMs < HOLD_CENTER_INTERVAL_MS) {
    vTaskDelay(pdMS_TO_TICKS(20));
    waitedMs += 20;
  }
}

static bool runCvSegment(float fromV, float toV, uint32_t targetIntervalUs, uint32_t &sampleIndex, uint32_t progressBaseSteps, uint32_t progressSegmentSteps, uint32_t progressTotalSteps) {
  float step = fabsf(cvConfig.stepV);
  if (step < 0.0005f) step = 0.0005f;

  int sign = (toV >= fromV) ? 1 : -1;
  SweepDirection direction = (sign > 0) ? DIR_UP : DIR_DOWN;
  float potential = fromV;
  float dacV = potentialToDacVoltage(potential);
  uint32_t totalSteps = (uint32_t)ceilf(fabsf(toV - fromV) / step);
  uint32_t currentStep = 0;

  writeAD5680_safe(voltageToDACCode(dacV));
  uint64_t nextDacUpdateUs = esp_timer_get_time() + targetIntervalUs;

  // DAC updates are checked before every ADC sample. While DAC is held, ADC is
  // sampled continuously as fast as the ADC/SPI/averaging path can complete.
  while (measurementEnabled && !measurementStopRequested) {
    uint64_t nowUs = esp_timer_get_time();

    while (currentStep < totalSteps && nowUs >= nextDacUpdateUs) {
      currentStep++;
      if (currentStep >= totalSteps) {
        potential = toV;
      } else {
        potential = fromV + (sign * step * currentStep);
      }

      dacV = potentialToDacVoltage(potential);
      writeAD5680_safe(voltageToDACCode(dacV));
      nextDacUpdateUs += targetIntervalUs;
      nowUs = esp_timer_get_time();
    }

    measureAdcAtCurrentDac(MEAS_MODE_CV, dacV, direction, sampleIndex);
    setMeasurementProgress(progressBaseSteps + min(currentStep, progressSegmentSteps), progressTotalSteps);

    if (currentStep >= totalSteps) {
      return true;
    }

    vTaskDelay(1);
  }

  return false;
}

static void runCvMode(uint32_t &sampleIndex) {
  CvConfig cfg = cvConfig;
  cvConfig.startV = clampFloat(cfg.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  cvConfig.vertex1V = clampFloat(cfg.vertex1V, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  cvConfig.vertex2V = clampFloat(cfg.vertex2V, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  cvConfig.finalV = clampFloat(cfg.finalV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  cvConfig.stepV = clampFloat(fabsf(cfg.stepV), 0.0005f, 0.25f);
  cvConfig.scanRateVps = clampFloat(fabsf(cfg.scanRateVps), 0.001f, 10.0f);
  if (cvConfig.cycles == 0) cvConfig.cycles = 1;
  if (cvConfig.cycles > 100) cvConfig.cycles = 100;

#if ENABLE_SD_LOGGING
  if (!initSdCard()) {
    Serial.println("CV aborted: SD_MMC is not available for .bin logging");
    requestMeasurementStop();
    return;
  }
#endif

  uint64_t startedUs = esp_timer_get_time();
  makeCvHeader(activeCvHeader, cvConfig, startedUs);
  makeCvFilePath(activeCvFilePath, sizeof(activeCvFilePath), startedUs, cvConfig.experimentName);
  sdRecordWriteCount = 0;
  sdRecordDropCount = 0;
  sdWriteErrorCount = 0;
  cvLogActive = true;

  Serial.print("CV started file=");
  Serial.println(activeCvFilePath);

  writeAD5680_safe(voltageToDACCode(potentialToDacVoltage(cvConfig.startV)));
  if (cvConfig.quietMs > 0) {
    uint32_t waitedMs = 0;
    while (measurementEnabled && !measurementStopRequested && waitedMs < cvConfig.quietMs) {
      vTaskDelay(pdMS_TO_TICKS(20));
      waitedMs += 20;
    }
  }

  uint32_t targetIntervalUs = (uint32_t)((cvConfig.stepV / cvConfig.scanRateVps) * 1000000.0f);
  if (targetIntervalUs < 1000) targetIntervalUs = 1000;

  uint32_t seg1Steps = (uint32_t)ceilf(fabsf(cvConfig.vertex1V - cvConfig.startV) / cvConfig.stepV);
  uint32_t seg2Steps = (uint32_t)ceilf(fabsf(cvConfig.vertex2V - cvConfig.vertex1V) / cvConfig.stepV);
  uint32_t seg3Steps = (uint32_t)ceilf(fabsf(cvConfig.finalV - cvConfig.vertex2V) / cvConfig.stepV);
  uint32_t stepsPerCycle = seg1Steps + seg2Steps + seg3Steps;
  uint32_t totalProgressSteps = stepsPerCycle * cvConfig.cycles;
  if (totalProgressSteps == 0) totalProgressSteps = 1;
  measurementCycleTotal = cvConfig.cycles;
  measurementProgressPermille = 0;

  bool cvCompleted = true;
  for (uint16_t cycle = 0; cycle < cvConfig.cycles && measurementEnabled && !measurementStopRequested; cycle++) {
    measurementCycleCurrent = cycle + 1;
    uint32_t cycleBaseSteps = (uint32_t)cycle * stepsPerCycle;
    Serial.print("CV cycle ");
    Serial.print(cycle + 1);
    Serial.print("/");
    Serial.println(cvConfig.cycles);

    if (!runCvSegment(cvConfig.startV, cvConfig.vertex1V, targetIntervalUs, sampleIndex, cycleBaseSteps, seg1Steps, totalProgressSteps)) {
      cvCompleted = false;
      break;
    }
    if (!runCvSegment(cvConfig.vertex1V, cvConfig.vertex2V, targetIntervalUs, sampleIndex, cycleBaseSteps + seg1Steps, seg2Steps, totalProgressSteps)) {
      cvCompleted = false;
      break;
    }
    if (!runCvSegment(cvConfig.vertex2V, cvConfig.finalV, targetIntervalUs, sampleIndex, cycleBaseSteps + seg1Steps + seg2Steps, seg3Steps, totalProgressSteps)) {
      cvCompleted = false;
      break;
    }
  }

  cvLogActive = false;
  measurementStopRequested = true;
  measurementEnabled = false;
  setMeasurementPriority(false);

  Serial.println(cvCompleted ? "CV completed" : "CV stopped before completion");
  measurementCompleted = cvCompleted;
  if (cvCompleted) measurementProgressPermille = 1000;
}

static void waitUntilUs(uint64_t targetUs) {
  while (measurementEnabled && !measurementStopRequested) {
    uint64_t nowUs = esp_timer_get_time();
    if (nowUs >= targetUs) break;

    uint64_t remainingUs = targetUs - nowUs;
    if (remainingUs > 2000) {
      vTaskDelay(1);
    } else if (remainingUs > 100) {
      delayMicroseconds(50);
    } else {
      delayMicroseconds((uint32_t)remainingUs);
    }
  }
}

static uint32_t pulseTailAverageWindowUs(uint32_t holdUs) {
  if (holdUs <= PULSE_TAIL_AVG_MIN_US) return holdUs;

  uint32_t tailUs = PULSE_TAIL_AVG_US;
  if (tailUs > holdUs) tailUs = holdUs;

  // Keep the first part of the pulse as settle time when the pulse is long enough.
  if (holdUs > (PULSE_TAIL_AVG_MIN_US * 2UL) && tailUs > (holdUs / 2UL)) {
    tailUs = holdUs / 2UL;
  }

  if (tailUs < PULSE_TAIL_AVG_MIN_US) tailUs = PULSE_TAIL_AVG_MIN_US;
  return tailUs;
}

static uint16_t readAD7694TailAverageUntilUs(uint64_t endUs) {
  uint64_t sum = 0;
  uint32_t count = 0;

  while (measurementEnabled && !measurementStopRequested) {
    uint64_t nowUs = esp_timer_get_time();
    if (nowUs >= endUs) break;

    sum += readAD7694Average_safe(ADC_AVG_SAMPLES);
    count++;

    if ((count & 0x0F) == 0) {
      taskYIELD();
    }
  }

  if (count == 0) {
    return readAD7694Average_safe(ADC_AVG_SAMPLES);
  }

  return (uint16_t)((sum + (count / 2UL)) / count);
}

static char readCurrentAtPotential(float potentialV, uint32_t holdUs, uint16_t &adcRaw, float &adcVoltage, float &currentAmp, float &rtiaOhm, uint8_t &tiaCN) {
  float dacV = potentialToDacVoltage(potentialV);
  writeAD5680_safe(voltageToDACCode(dacV));

  uint64_t endUs = esp_timer_get_time() + holdUs;
  uint32_t tailUs = pulseTailAverageWindowUs(holdUs);
  waitUntilUs(endUs - tailUs);

  adcRaw = readAD7694TailAverageUntilUs(endUs);
  adcVoltage = adcRawToVoltage(adcRaw);

  char action = autoRangeUpdateAndMaybeChange(adcVoltage);
  if (action != '-') {
    delayMicroseconds(AUTO_RANGE_SETTLE_US);
    adcRaw = readAD7694Average_safe(ADC_AVG_SAMPLES);
    adcVoltage = adcRawToVoltage(adcRaw);
  }

  float adcDeltaVoltage = adcVoltage - ADC_ZERO_VOLTAGE;
  rtiaOhm = getCurrentRtiaOhm();
  tiaCN = getCurrentTIACN();
  currentAmp = adcDeltaVoltage / rtiaOhm;
  return action;
}

static uint8_t motorPercentToDuty(uint8_t percent) {
  if (percent > 100) percent = 100;
  return (uint8_t)((uint16_t)percent * 255U / 100U);
}

static bool runSwvConditioning() {
  if (!swvConfig.conditioningEnabled || swvConfig.conditioningMs == 0) return true;

  float conditioningDacV = potentialToDacVoltage(swvConfig.conditioningV);
  writeAD5680_safe(voltageToDACCode(conditioningDacV));
  vibrationOff();

  Serial.print("SWV conditioning ");
  Serial.print(swvConfig.conditioningV, 3);
  Serial.print(" V for ");
  Serial.print(swvConfig.conditioningMs);
  Serial.println(" ms");

  uint64_t endUs = esp_timer_get_time() + ((uint64_t)swvConfig.conditioningMs * 1000ULL);
  bool motorOn = false;
  uint64_t nextToggleUs = esp_timer_get_time();
  uint8_t motorDuty = motorPercentToDuty(swvConfig.motorPowerPercent);

  while (measurementEnabled && !measurementStopRequested && esp_timer_get_time() < endUs) {
    uint64_t nowUs = esp_timer_get_time();

    if (swvConfig.agitationEnabled) {
      if (nowUs >= nextToggleUs) {
        motorOn = !motorOn;
        if (motorOn) {
          vibrationOn(motorDuty);
          nextToggleUs = nowUs + ((uint64_t)swvConfig.agitationOnMs * 1000ULL);
        } else {
          vibrationOff();
          nextToggleUs = nowUs + ((uint64_t)swvConfig.agitationOffMs * 1000ULL);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  vibrationOff();
  if (!measurementEnabled || measurementStopRequested) return false;

  if (swvConfig.settleAfterMs > 0) {
    uint32_t waitedMs = 0;
    while (measurementEnabled && !measurementStopRequested && waitedMs < swvConfig.settleAfterMs) {
      vTaskDelay(pdMS_TO_TICKS(20));
      waitedMs += 20;
    }
  }

  return measurementEnabled && !measurementStopRequested;
}

static bool runDpvConditioning() {
  if (!dpvConfig.conditioningEnabled || dpvConfig.conditioningMs == 0) return true;

  float conditioningDacV = potentialToDacVoltage(dpvConfig.conditioningV);
  writeAD5680_safe(voltageToDACCode(conditioningDacV));
  vibrationOff();

  Serial.print("DPV conditioning ");
  Serial.print(dpvConfig.conditioningV, 3);
  Serial.print(" V for ");
  Serial.print(dpvConfig.conditioningMs);
  Serial.println(" ms");

  uint64_t endUs = esp_timer_get_time() + ((uint64_t)dpvConfig.conditioningMs * 1000ULL);
  bool motorOn = false;
  uint64_t nextToggleUs = esp_timer_get_time();
  uint8_t motorDuty = motorPercentToDuty(dpvConfig.motorPowerPercent);

  while (measurementEnabled && !measurementStopRequested && esp_timer_get_time() < endUs) {
    uint64_t nowUs = esp_timer_get_time();

    if (dpvConfig.agitationEnabled) {
      if (nowUs >= nextToggleUs) {
        motorOn = !motorOn;
        if (motorOn) {
          vibrationOn(motorDuty);
          nextToggleUs = nowUs + ((uint64_t)dpvConfig.agitationOnMs * 1000ULL);
        } else {
          vibrationOff();
          nextToggleUs = nowUs + ((uint64_t)dpvConfig.agitationOffMs * 1000ULL);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }

  vibrationOff();
  if (!measurementEnabled || measurementStopRequested) return false;

  if (dpvConfig.settleAfterMs > 0) {
    uint32_t waitedMs = 0;
    while (measurementEnabled && !measurementStopRequested && waitedMs < dpvConfig.settleAfterMs) {
      vTaskDelay(pdMS_TO_TICKS(20));
      waitedMs += 20;
    }
  }

  return measurementEnabled && !measurementStopRequested;
}

static void runSwvMode(uint32_t &sampleIndex) {
  SwvConfig cfg = swvConfig;
  swvConfig.startV = clampFloat(cfg.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  swvConfig.endV = clampFloat(cfg.endV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  swvConfig.stepV = clampFloat(fabsf(cfg.stepV), 0.0005f, 0.25f);
  swvConfig.amplitudeV = clampFloat(fabsf(cfg.amplitudeV), 0.0005f, 1.0f);
  swvConfig.frequencyHz = clampFloat(fabsf(cfg.frequencyHz), 0.1f, 500.0f);
  swvConfig.periodMs = clampFloat(fabsf(cfg.periodMs), 2.0f, 10000.0f);
  swvConfig.useFrequency = cfg.useFrequency;
  swvConfig.useDutyPercent = cfg.useDutyPercent;
  swvConfig.frequencyHz = swvConfig.useFrequency ? swvConfig.frequencyHz : (1000.0f / swvConfig.periodMs);
  swvConfig.periodMs = 1000.0f / swvConfig.frequencyHz;
  swvConfig.dutyPercent = clampFloat(fabsf(cfg.dutyPercent), 1.0f, 99.0f);
  swvConfig.dutyMs = fabsf(cfg.dutyMs);
  if (swvConfig.useDutyPercent) {
    swvConfig.dutyMs = swvConfig.periodMs * swvConfig.dutyPercent * 0.01f;
  } else {
    swvConfig.dutyMs = clampFloat(swvConfig.dutyMs, 0.1f, swvConfig.periodMs - 0.1f);
    swvConfig.dutyPercent = (swvConfig.dutyMs / swvConfig.periodMs) * 100.0f;
  }
  swvConfig.conditioningEnabled = cfg.conditioningEnabled;
  swvConfig.conditioningV = cfg.conditioningV;
  swvConfig.conditioningMs = cfg.conditioningMs;
  swvConfig.agitationEnabled = cfg.agitationEnabled;
  swvConfig.agitationOnMs = cfg.agitationOnMs;
  swvConfig.agitationOffMs = cfg.agitationOffMs;
  swvConfig.motorPowerPercent = cfg.motorPowerPercent;
  swvConfig.settleAfterMs = cfg.settleAfterMs;
  swvConfig.conditioningV = clampFloat(cfg.conditioningV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  if (swvConfig.conditioningMs > 600000UL) swvConfig.conditioningMs = 600000UL;
  if (swvConfig.agitationOnMs < 100UL) swvConfig.agitationOnMs = 100UL;
  if (swvConfig.agitationOffMs < 100UL) swvConfig.agitationOffMs = 100UL;
  if (swvConfig.motorPowerPercent > 100) swvConfig.motorPowerPercent = 100;
  if (swvConfig.settleAfterMs > 60000UL) swvConfig.settleAfterMs = 60000UL;
  if (swvConfig.quietMs > 600000UL) swvConfig.quietMs = 600000UL;

#if ENABLE_SD_LOGGING
  if (!initSdCard()) {
    Serial.println("SWV aborted: SD_MMC is not available for .bin logging");
    requestMeasurementStop();
    return;
  }
#endif

  uint64_t startedUs = esp_timer_get_time();
  makeSwvHeader(activeCvHeader, swvConfig, startedUs);
  makeMeasurementFilePath("swv", activeCvFilePath, sizeof(activeCvFilePath), startedUs, swvConfig.experimentName);
  sdRecordWriteCount = 0;
  sdRecordDropCount = 0;
  sdWriteErrorCount = 0;

  Serial.print("SWV started file=");
  Serial.println(activeCvFilePath);

  if (!runSwvConditioning()) {
    vibrationOff();
    measurementStopRequested = true;
    measurementEnabled = false;
    setMeasurementPriority(false);
    Serial.println("SWV stopped during conditioning");
    return;
  }

  cvLogActive = true;

  writeAD5680_safe(voltageToDACCode(potentialToDacVoltage(swvConfig.startV)));
  if (swvConfig.quietMs > 0) {
    uint32_t waitedMs = 0;
    while (measurementEnabled && !measurementStopRequested && waitedMs < swvConfig.quietMs) {
      vTaskDelay(pdMS_TO_TICKS(20));
      waitedMs += 20;
    }
  }

  int sign = (swvConfig.endV >= swvConfig.startV) ? 1 : -1;
  SweepDirection direction = (sign > 0) ? DIR_UP : DIR_DOWN;
  uint32_t totalSteps = (uint32_t)floorf(fabsf(swvConfig.endV - swvConfig.startV) / swvConfig.stepV) + 1;
  if (totalSteps == 0) totalSteps = 1;
  measurementCycleCurrent = 1;
  measurementCycleTotal = 1;
  measurementProgressPermille = 0;
  uint32_t forwardHoldUs = (uint32_t)(swvConfig.dutyMs * 1000.0f);
  uint32_t reverseHoldUs = (uint32_t)((swvConfig.periodMs - swvConfig.dutyMs) * 1000.0f);
  if (forwardHoldUs < 100) forwardHoldUs = 100;
  if (reverseHoldUs < 100) reverseHoldUs = 100;

  bool swvCompleted = true;
  for (uint32_t stepIndex = 0; stepIndex < totalSteps && measurementEnabled && !measurementStopRequested; stepIndex++) {
    uint64_t t0 = esp_timer_get_time();
    float basePotential = swvConfig.startV + (sign * swvConfig.stepV * stepIndex);
    if ((sign > 0 && basePotential > swvConfig.endV) || (sign < 0 && basePotential < swvConfig.endV)) {
      basePotential = swvConfig.endV;
    }

    float halfAmplitude = swvConfig.amplitudeV * 0.5f;
    float forwardPotential = clampFloat(basePotential + (sign * halfAmplitude), CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
    float reversePotential = clampFloat(basePotential - (sign * halfAmplitude), CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);

    uint16_t forwardRaw = 0;
    uint16_t reverseRaw = 0;
    float forwardAdcV = 0.0f;
    float reverseAdcV = 0.0f;
    float forwardCurrentA = 0.0f;
    float reverseCurrentA = 0.0f;
    float rtiaOhm = getCurrentRtiaOhm();
    uint8_t tiaCN = getCurrentTIACN();
    char action = '-';

    action = readCurrentAtPotential(forwardPotential, forwardHoldUs, forwardRaw, forwardAdcV, forwardCurrentA, rtiaOhm, tiaCN);
    char reverseAction = readCurrentAtPotential(reversePotential, reverseHoldUs, reverseRaw, reverseAdcV, reverseCurrentA, rtiaOhm, tiaCN);
    if (action == '-') action = reverseAction;

    MeasurementData data;
    memset(&data, 0, sizeof(data));
    data.mode = MEAS_MODE_SWV;
    data.sampleIndex = sampleIndex++;
    data.direction = direction;
    data.dacSetVoltage = potentialToDacVoltage(basePotential);
    data.dacCode = voltageToDACCode(data.dacSetVoltage);
    data.dacExpectedVoltage = dacCodeToVoltage(data.dacCode);
    data.adcRaw = forwardRaw;
    data.adcVoltage = forwardAdcV;
    data.adcDeltaVoltage = (forwardAdcV - reverseAdcV);
    data.currentAmp = forwardCurrentA - reverseCurrentA;
    data.rtiaOhm = rtiaOhm;
    data.tiaCN = tiaCN;
    data.autoRangeAction = action;
    data.timestamp_us = esp_timer_get_time();
    data.period_us = (uint32_t)(data.timestamp_us - t0);
    publishMeasurement(data);
    setMeasurementProgress(stepIndex + 1, totalSteps);

    vTaskDelay(1);
  }

  if (measurementStopRequested) swvCompleted = false;

  cvLogActive = false;
  measurementStopRequested = true;
  measurementEnabled = false;
  setMeasurementPriority(false);

  Serial.println(swvCompleted ? "SWV completed" : "SWV stopped before completion");
  measurementCompleted = swvCompleted;
  if (swvCompleted) measurementProgressPermille = 1000;
}

static void runDpvMode(uint32_t &sampleIndex) {
  DpvConfig cfg = dpvConfig;
  dpvConfig.startV = clampFloat(cfg.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  dpvConfig.endV = clampFloat(cfg.endV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  dpvConfig.stepV = clampFloat(fabsf(cfg.stepV), 0.0005f, 0.25f);
  dpvConfig.amplitudeV = clampFloat(fabsf(cfg.amplitudeV), 0.0005f, 1.0f);
  dpvConfig.periodMs = clampFloat(fabsf(cfg.periodMs), 2.0f, 10000.0f);
  dpvConfig.pulseMs = clampFloat(fabsf(cfg.pulseMs), 0.1f, dpvConfig.periodMs - 0.1f);
  dpvConfig.conditioningEnabled = cfg.conditioningEnabled;
  dpvConfig.conditioningV = clampFloat(cfg.conditioningV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  dpvConfig.conditioningMs = cfg.conditioningMs;
  dpvConfig.agitationEnabled = cfg.agitationEnabled;
  dpvConfig.agitationOnMs = cfg.agitationOnMs;
  dpvConfig.agitationOffMs = cfg.agitationOffMs;
  dpvConfig.motorPowerPercent = cfg.motorPowerPercent;
  dpvConfig.settleAfterMs = cfg.settleAfterMs;
  if (dpvConfig.conditioningMs > 600000UL) dpvConfig.conditioningMs = 600000UL;
  if (dpvConfig.agitationOnMs < 100UL) dpvConfig.agitationOnMs = 100UL;
  if (dpvConfig.agitationOffMs < 100UL) dpvConfig.agitationOffMs = 100UL;
  if (dpvConfig.motorPowerPercent > 100) dpvConfig.motorPowerPercent = 100;
  if (dpvConfig.settleAfterMs > 60000UL) dpvConfig.settleAfterMs = 60000UL;
  if (dpvConfig.quietMs > 600000UL) dpvConfig.quietMs = 600000UL;

#if ENABLE_SD_LOGGING
  if (!initSdCard()) {
    Serial.println("DPV aborted: SD_MMC is not available for .bin logging");
    requestMeasurementStop();
    return;
  }
#endif

  uint64_t startedUs = esp_timer_get_time();
  makeDpvHeader(activeCvHeader, dpvConfig, startedUs);
  makeMeasurementFilePath("dpv", activeCvFilePath, sizeof(activeCvFilePath), startedUs, dpvConfig.experimentName);
  sdRecordWriteCount = 0;
  sdRecordDropCount = 0;
  sdWriteErrorCount = 0;

  Serial.print("DPV started file=");
  Serial.println(activeCvFilePath);

  if (!runDpvConditioning()) {
    vibrationOff();
    measurementStopRequested = true;
    measurementEnabled = false;
    setMeasurementPriority(false);
    Serial.println("DPV stopped during conditioning");
    return;
  }

  cvLogActive = true;

  writeAD5680_safe(voltageToDACCode(potentialToDacVoltage(dpvConfig.startV)));
  if (dpvConfig.quietMs > 0) {
    uint32_t waitedMs = 0;
    while (measurementEnabled && !measurementStopRequested && waitedMs < dpvConfig.quietMs) {
      vTaskDelay(pdMS_TO_TICKS(20));
      waitedMs += 20;
    }
  }

  int sign = (dpvConfig.endV >= dpvConfig.startV) ? 1 : -1;
  SweepDirection direction = (sign > 0) ? DIR_UP : DIR_DOWN;
  uint32_t totalSteps = (uint32_t)floorf(fabsf(dpvConfig.endV - dpvConfig.startV) / dpvConfig.stepV) + 1;
  if (totalSteps == 0) totalSteps = 1;
  measurementCycleCurrent = 1;
  measurementCycleTotal = 1;
  measurementProgressPermille = 0;
  uint32_t pulseUs = (uint32_t)(dpvConfig.pulseMs * 1000.0f);
  uint32_t baseHoldUs = (uint32_t)((dpvConfig.periodMs - dpvConfig.pulseMs) * 1000.0f);
  if (pulseUs < 100) pulseUs = 100;
  if (baseHoldUs < 100) baseHoldUs = 100;

  bool dpvCompleted = true;
  for (uint32_t stepIndex = 0; stepIndex < totalSteps && measurementEnabled && !measurementStopRequested; stepIndex++) {
    uint64_t t0 = esp_timer_get_time();
    float basePotential = dpvConfig.startV + (sign * dpvConfig.stepV * stepIndex);
    if ((sign > 0 && basePotential > dpvConfig.endV) || (sign < 0 && basePotential < dpvConfig.endV)) {
      basePotential = dpvConfig.endV;
    }

    float pulsePotential = clampFloat(basePotential + (sign * dpvConfig.amplitudeV), CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);

    uint16_t baseRaw = 0;
    uint16_t pulseRaw = 0;
    float baseAdcV = 0.0f;
    float pulseAdcV = 0.0f;
    float baseCurrentA = 0.0f;
    float pulseCurrentA = 0.0f;
    float rtiaOhm = getCurrentRtiaOhm();
    uint8_t tiaCN = getCurrentTIACN();
    char action = '-';

    action = readCurrentAtPotential(basePotential, baseHoldUs, baseRaw, baseAdcV, baseCurrentA, rtiaOhm, tiaCN);
    char pulseAction = readCurrentAtPotential(pulsePotential, pulseUs, pulseRaw, pulseAdcV, pulseCurrentA, rtiaOhm, tiaCN);
    if (action == '-') action = pulseAction;

    MeasurementData data;
    memset(&data, 0, sizeof(data));
    data.mode = MEAS_MODE_DPV;
    data.sampleIndex = sampleIndex++;
    data.direction = direction;
    data.dacSetVoltage = potentialToDacVoltage(basePotential);
    data.dacCode = voltageToDACCode(data.dacSetVoltage);
    data.dacExpectedVoltage = dacCodeToVoltage(data.dacCode);
    data.adcRaw = pulseRaw;
    data.adcVoltage = pulseAdcV;
    data.adcDeltaVoltage = (pulseAdcV - baseAdcV);
    data.currentAmp = pulseCurrentA - baseCurrentA;
    data.rtiaOhm = rtiaOhm;
    data.tiaCN = tiaCN;
    data.autoRangeAction = action;
    data.timestamp_us = esp_timer_get_time();
    data.period_us = (uint32_t)(data.timestamp_us - t0);
    publishMeasurement(data);
    setMeasurementProgress(stepIndex + 1, totalSteps);

    vTaskDelay(1);
  }

  if (measurementStopRequested) dpvCompleted = false;

  cvLogActive = false;
  measurementStopRequested = true;
  measurementEnabled = false;
  setMeasurementPriority(false);

  Serial.println(dpvCompleted ? "DPV completed" : "DPV stopped before completion");
  measurementCompleted = dpvCompleted;
  if (dpvCompleted) measurementProgressPermille = 1000;
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

    MeasurementMode mode = getMeasurementMode();
    Serial.print("Measurement started mode=");
    Serial.println(measurementModeToText(mode));

    while (measurementEnabled && !measurementStopRequested) {
      mode = getMeasurementMode();

      if (mode == MEAS_MODE_HOLD_CENTER) {
        runHoldCenterMode(sampleIndex);
      } else if (mode == MEAS_MODE_CV) {
        runCvMode(sampleIndex);
      } else if (mode == MEAS_MODE_SWV) {
        runSwvMode(sampleIndex);
      } else if (mode == MEAS_MODE_DPV) {
        runDpvMode(sampleIndex);
      } else {
        runSweepMode(sampleIndex);
      }

      if (measurementEnabled && !measurementStopRequested) {
        setMeasurementPriority(true);
      }
    }

    measurementBusy = false;
    measurementEnabled = false;
    measurementStopRequested = true;
    setMeasurementPriority(false);
    tftRunStatus = measurementCompleted ? TFT_RUN_COMPLETED : TFT_RUN_STOPPED;
    requestTftFullRefresh();

    Serial.println("Measurement stopped, task blocked until next RUN");
  }
}

#if ENABLE_TFT_DISPLAY
static const uint16_t TFT_BG        = 0x0841;
static const uint16_t TFT_SURFACE   = 0x1082;
static const uint16_t TFT_SURFACE_2 = 0x18E3;
static const uint16_t TFT_LINE      = 0x39E7;
static const uint16_t TFT_MUTED     = 0x9CD3;
static const uint16_t TFT_TEXT      = 0xFFFF;
static const uint16_t TFT_ACCENT    = 0x0579;
static const uint16_t TFT_ACCENT_2  = 0x5D9F;
static const uint16_t TFT_OK_COLOR  = 0x07E0;
static const uint16_t TFT_WARN_COLOR = 0xFD20;

static String tftFitText(String text, uint8_t maxChars) {
  if (text.length() <= maxChars) return text;
  if (maxChars <= 3) return text.substring(0, maxChars);
  return text.substring(0, maxChars - 3) + "...";
}

static void tftCard(int16_t x, int16_t y, int16_t w, int16_t h, const char *title) {
  lcd.fillRoundRect(x, y, w, h, 6, TFT_SURFACE);
  lcd.drawRoundRect(x, y, w, h, 6, TFT_LINE);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_MUTED, TFT_SURFACE);
  lcd.setCursor(x + 10, y + 8);
  lcd.print(title);
}

static void tftCardLine(int16_t x, int16_t y, const String &text, uint16_t color = TFT_TEXT, uint16_t bg = TFT_SURFACE) {
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(color, bg);
  lcd.setCursor(x, y);
  lcd.print(text);
}

static void tftValue(int16_t x, int16_t y, const String &label, const String &value, uint16_t color = TFT_TEXT) {
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_MUTED, TFT_SURFACE);
  lcd.setCursor(x, y);
  lcd.print(label);
  lcd.setTextColor(color, TFT_SURFACE);
  lcd.setCursor(x, y + 16);
  lcd.print(value);
}

static const char *tftRunStatusLabel() {
  switch (tftRunStatus) {
    case TFT_RUN_STARTING:  return "STARTING";
    case TFT_RUN_COMPLETED: return "COMPLETED";
    case TFT_RUN_STOPPED:   return "STOPPED";
    case TFT_RUN_READY:
    default:                return "READY";
  }
}

static uint16_t tftRunStatusColor() {
  switch (tftRunStatus) {
    case TFT_RUN_STARTING:  return TFT_ACCENT_2;
    case TFT_RUN_COMPLETED: return TFT_OK_COLOR;
    case TFT_RUN_STOPPED:   return TFT_WARN_COLOR;
    case TFT_RUN_READY:
    default:                return TFT_OK_COLOR;
  }
}

static String tftRunStatusDetail() {
  switch (tftRunStatus) {
    case TFT_RUN_STARTING:
      return String("Starting ") + measurementModeToText(getMeasurementMode()) + " measurement";
    case TFT_RUN_COMPLETED:
      return String(measurementModeToText(getMeasurementMode())) + " finished; data available in web app";
    case TFT_RUN_STOPPED:
      return "Measurement stopped; GLCD and touch are active";
    case TFT_RUN_READY:
    default:
      return "Ready for measurement";
  }
}

static void drawRunStatusCard() {
  tftCard(10, 266, 460, 34, "MEASUREMENT STATUS");
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(tftRunStatusColor(), TFT_SURFACE);
  lcd.setCursor(22, 288);
  lcd.print(tftRunStatusLabel());
  tftCardLine(104, 288, tftFitText(tftRunStatusDetail(), 48), TFT_TEXT);
}

static void showTftRunStatusImmediate() {
  if (measurementSessionActive()) return;
  if (spiMutex == nullptr) return;
  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(250)) != pdTRUE) return;

  deselectAllSPI();
  lcd.setRotation(1);
  lcd.startWrite();
  drawRunStatusCard();
  lcd.endWrite();
  deselectAllSPI();

  xSemaphoreGive(spiMutex);
}

void drawFirmwareVersion() {
  // Must be called inside an active lcd.startWrite() block.
  String text = String("FW ") + FW_VERSION;
  int16_t x = lcd.width() - 8;
  int16_t y = lcd.height() - 8;

  lcd.setTextDatum(BR_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_MUTED, TFT_BG);
  lcd.fillRect(lcd.width() - 100, lcd.height() - 18, 100, 18, TFT_BG);
  lcd.drawString(text, x, y);
  lcd.setTextDatum(TL_DATUM);
}

void drawOtaStatusScreen(const String &title, const String &line1, const String &line2, uint16_t color) {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.setRotation(1);
  lcd.startWrite();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(TL_DATUM);

  lcd.setTextSize(2);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(10, 16);
  lcd.println("smart-ec");

  lcd.setTextColor(color, TFT_BLACK);
  lcd.setCursor(10, 72);
  lcd.println(title);

  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(10, 126);
  lcd.println(line1);
  lcd.setCursor(10, 146);
  lcd.println(line2);
  lcd.setCursor(10, 186);
  lcd.println("Please keep power connected.");

  drawFirmwareVersion();

  lcd.endWrite();
  deselectAllSPI();
}

void showOtaStatusScreen(const String &title, const String &line1, const String &line2, uint16_t color) {
  if (!takeSPI(1000)) {
    Serial.println("OTA status screen skipped: spiMutex timeout");
    return;
  }
  drawOtaStatusScreen(title, line1, line2, color);
  giveSPI();
}

void printRtcTimeLine(const String &line) {
  lcd.fillRect(257, 240, 200, 14, TFT_SURFACE);
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_MUTED, TFT_SURFACE);
  lcd.drawString(tftFitText(line, 31), 257, 240);
  lcd.setTextDatum(TL_DATUM);
}

void drawRtcTimeLine() {
  // Must be called while holding spiMutex.
  String timeLine = buildRtcTimeLineText();

  deselectAllSPI();

  lcd.setRotation(1);
  lcd.startWrite();
  lcd.setTextDatum(TL_DATUM);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextSize(1);
  printRtcTimeLine(timeLine);

  lcd.endWrite();
  deselectAllSPI();
}

void drawStatusPanel() {
  // Must be called while holding spiMutex.
  TftStatusText text = buildTftStatusText();

  deselectAllSPI();

  lcd.setRotation(1);
  lcd.startWrite();
  lcd.setTextDatum(TL_DATUM);

  lcd.fillRect(8, 76, 464, 236, TFT_BG);

  tftCard(10, 78, 460, 108, "NETWORK");
  tftCardLine(22, 104, tftFitText(text.wifiLine1, 54));
  tftCardLine(22, 122, text.wifiLine2, TFT_ACCENT_2);
  tftCardLine(22, 140, tftFitText(text.wifiLine3, 54), TFT_MUTED);

  tftCard(10, 196, 225, 64, "STORAGE");
  tftCardLine(22, 222, tftFitText(text.sdLine, 31), sdMounted ? TFT_OK_COLOR : TFT_WARN_COLOR);
  tftCardLine(22, 240, tftFitText(text.stateLine, 31), measurementEnabled ? TFT_WARN_COLOR : TFT_OK_COLOR);

  tftCard(245, 196, 225, 64, "RTC");
  tftCardLine(257, 222, tftFitText(text.rtcLine, 31), rtcUtcSynced ? TFT_OK_COLOR : TFT_WARN_COLOR);
  tftCardLine(257, 240, tftFitText(text.rtcTimeLine, 31), TFT_MUTED);

  drawRunStatusCard();
  drawFirmwareVersion();

  lcd.endWrite();
  deselectAllSPI();
}

void drawStaticScreen() {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.setRotation(1);                 // Set rotation before clearing/drawing.
  lcd.startWrite();
  lcd.fillScreen(TFT_BG);
  lcd.setTextDatum(TL_DATUM);

  lcd.fillRect(0, 0, lcd.width(), 68, TFT_SURFACE);
  lcd.drawFastHLine(0, 68, lcd.width(), TFT_LINE);

  lcd.fillRoundRect(12, 10, 318, 45, 6, TFT_SURFACE_2);
  lcd.drawRoundRect(12, 10, 318, 45, 6, TFT_LINE);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_MUTED, TFT_SURFACE_2);
  lcd.setCursor(24, 18);
  lcd.print("MODE");
  lcd.setTextSize(2);
  lcd.setTextColor(TFT_TEXT, TFT_SURFACE_2);
  lcd.setCursor(24, 32);
  lcd.print(measurementModeToText(getMeasurementMode()));

  lcd.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, TFT_LINE);
  drawFirmwareVersion();

  lcd.endWrite();
  drawStatusPanel();
  deselectAllSPI();
}

void drawRunButton(bool running) {
  // Must be called while holding spiMutex.
  deselectAllSPI();

  lcd.setRotation(1);

  if (!running && LittleFS.exists(RUN_LOGO_FILE)) {
    lcd.startWrite();
    lcd.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, TFT_BG);
    lcd.endWrite();

    if (lcd.drawPngFile(LittleFS, RUN_LOGO_FILE, BTN_X, BTN_Y, BTN_W, BTN_H)) {
      deselectAllSPI();
      return;
    }
  }

  lcd.startWrite();
  uint16_t bg = running ? TFT_RED : TFT_OK_COLOR;
  lcd.fillRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, bg);
  lcd.drawRoundRect(BTN_X, BTN_Y, BTN_W, BTN_H, 6, TFT_TEXT);
  lcd.setTextColor(running ? TFT_WHITE : TFT_BLACK, bg);
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

  lcd.setRotation(1);
  lcd.startWrite();
  lcd.setTextDatum(TL_DATUM);

  lcd.fillRect(8, 76, 464, 236, TFT_BG);

  tftCard(10, 78, 460, 48, "LATEST SAMPLE");
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_ACCENT_2, TFT_SURFACE);
  lcd.setCursor(22, 104);
  lcd.print(measurementModeToText(data.mode));
  lcd.setTextColor(TFT_MUTED, TFT_SURFACE);
  lcd.setCursor(100, 104);
  lcd.print(directionToText(data.direction));
  lcd.setCursor(210, 104);
  lcd.print("Period ");
  lcd.setTextColor(TFT_TEXT, TFT_SURFACE);
  lcd.print(data.period_us);
  lcd.print(" us");

  tftCard(10, 136, 145, 62, "POTENTIAL");
  tftValue(22, 160, "DAC set", String(data.dacSetVoltage, 3) + " V", TFT_ACCENT_2);

  tftCard(168, 136, 145, 62, "ADC");
  tftValue(180, 160, "Voltage", String(data.adcVoltage, 6) + " V", TFT_ACCENT_2);

  tftCard(325, 136, 145, 62, "CURRENT");
  tftValue(337, 160, "I", String(data.currentAmp * 1000000.0f, 3) + " uA", TFT_ACCENT_2);

  tftCard(10, 208, 225, 40, "RANGE");
  tftCardLine(22, 232, String("RTIA ") + String(data.rtiaOhm, 0) + " ohm  " + data.autoRangeAction);

  tftCard(245, 208, 225, 40, "RAW");
  tftCardLine(257, 232, String("ADC ") + String(data.adcRaw) + String("   DAC ") + String(data.dacCode));

  drawRunStatusCard();
  drawFirmwareVersion();

  lcd.endWrite();
  deselectAllSPI();
}

void serviceTftRefresh() {
  // Must be called only from optionTask.
  // GLCD and Touch share the SPI bus with ADC/DAC. During a measurement session
  // no display transaction is allowed, including quiet/settle windows.
  if (measurementSessionActive()) return;

  bool doFull = tftFullRefreshRequested;
  bool doStatus = tftStatusRefreshRequested;
  bool doTime = tftTimeRefreshRequested;
  bool doRun  = tftRunButtonRefreshRequested;
  bool doMeas = tftDrawMeasurementRequested && hasLastMeasurement;

  if (!doFull && !doStatus && !doTime && !doRun && !doMeas) return;

  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    return;
  }

  // Clear requests after we get the mutex, so a missed draw can be requested again.
  tftFullRefreshRequested = false;
  tftStatusRefreshRequested = false;
  tftTimeRefreshRequested = false;
  tftRunButtonRefreshRequested = false;
  tftDrawMeasurementRequested = false;

  deselectAllSPI();

  if (doFull) {
    drawStaticScreen();
  } else if (doStatus) {
    drawStatusPanel();
  } else if (doTime) {
    drawRtcTimeLine();
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
void showOtaStatusScreen(const String&, const String&, const String&, uint16_t) {}
#endif

// ======================================================
// Option task helpers: Serial / logger / I2C / manual TFT
// ======================================================
void printMeasurementCsv(const MeasurementData &data) {
  Serial.print(measurementModeToText(data.mode));
  Serial.print(',');
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
  while (measurementSessionActive()) {
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
  if (measurementSessionActive()) {
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

#if !ENABLE_TFT_DISPLAY
static void showTftRunStatusImmediate() {}
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
  if (measurementSessionActive()) return;

  uint32_t nowMs = millis();
  if (nowMs - lastTouchMs < 50) return;   // about 20 Hz
  lastTouchMs = nowMs;

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
  if (measurementSessionActive()) {
    Serial.println("Raw touch skipped: measurement active");
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
  if (measurementSessionActive()) {
    Serial.println("Touch read skipped: measurement active");
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

void printMeasurementMode() {
  Serial.print("Measurement mode=");
  Serial.println(measurementModeToText(getMeasurementMode()));
}

void selectMeasurementModeFromSerial(MeasurementMode mode) {
  if (!setMeasurementMode(mode)) {
    Serial.println("Mode change skipped: STOP measurement first");
    return;
  }

  printMeasurementMode();
  requestTftFullRefresh();
}

void handleSerialCommand(char c) {
  if (c == '\r' || c == '\n') return;

  if (c == 'r' || c == 'R') {
    if (measurementEnabled) {
      Serial.println("Measurement already running; send 's' first before starting a new run");
      return;
    }

    resetAutoRangeToLowestForNewRun();
    requestMeasurementRun();
    digitalWrite(LED_BUILTIN_PIN, LOW);
    refreshRunButtonFromOptionTask();
    Serial.print("Measurement RUN mode=");
    Serial.print(measurementModeToText(getMeasurementMode()));
    Serial.println(" from configured RTIA start range");
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
    printMeasurementMode();
    Serial.print("serialDebugMode=");
    Serial.println(serialDebugMode ? "true" : "false");
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
  else if (c == '1') {
    selectMeasurementModeFromSerial(MEAS_MODE_SWEEP);
  }
  else if (c == '2') {
    selectMeasurementModeFromSerial(MEAS_MODE_HOLD_CENTER);
  }
  else if (c == '3') {
    selectMeasurementModeFromSerial(MEAS_MODE_CV);
  }
  else if (c == '4') {
    selectMeasurementModeFromSerial(MEAS_MODE_SWV);
  }
  else if (c == '5') {
    selectMeasurementModeFromSerial(MEAS_MODE_DPV);
  }
  else if (c == 'm' || c == 'M') {
    printMeasurementMode();
    Serial.println("Modes: 1=SWEEP, 2=HOLD_CENTER, 3=CV, 4=SWV, 5=DPV");
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
static const char *OTA_WEB_VERSION_KEY  = "web_ver";
static const char *WIFI_ASSET_DIR       = "/device_assets";        // optional SD card assets served by the web app
static const char *WIFI_PLOTLY_FILE     = "/device_assets/plotly.min.js";
static const char *WIFI_PLOTLY_TEMP_FILE = "/device_assets/plotly.tmp";
static const char *WIFI_PLOTLY_LEGACY_FILE = "/plotly.min.js";
static const char *WIFI_AP_PASSWORD     = "12345678";
static const uint8_t WIFI_MAX_CREDS     = 12;
static const uint8_t WIFI_DEFAULT_ATTEMPTS = 3;
static const uint32_t WIFI_DEFAULT_TIMEOUT_MS = 6000;
static const uint32_t WIFI_STORED_TIMEOUT_MS  = 4500;
static const uint32_t WIFI_RETRY_PERIOD_MS    = 30000;
static const uint16_t WIFI_SCAN_MS_PER_CH     = 120;  // active scan; all channels about 1.5-2.0s

static const char *VERSION_URL = "https://github.com/tawee11/potenV2/releases/latest/download/version.json";
static const char *FIRMWARE_URL_FALLBACK = "https://github.com/tawee11/potenV2/releases/latest/download/firmware.bin";
static const char *FILESYSTEM_URL_FALLBACK = "https://github.com/tawee11/potenV2/releases/latest/download/littlefs.bin";

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
static File wifiAssetUploadFile;
static String installedWebVersion = WEB_VERSION;
static bool wifiServerStarted = false;
static bool wifiApMode = false;
static bool wifiMdnsStarted = false;
static bool wifiAssetUploadOk = false;
static size_t wifiAssetUploadBytes = 0;
static String wifiAssetUploadMessage;
static bool wifiConnectRequested = false;
static bool vibrationTestActive = false;
static uint32_t wifiLastRetryMs = 0;
static String wifiApSsid = "smart-ec-Setup";

static String wifiDeviceIdSuffix() {
  uint64_t mac = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06x", (uint32_t)(mac & 0xFFFFFF));
  return String(suffix);
}

static String wifiMdnsHost() {
  return String("smart-ec-") + wifiDeviceIdSuffix();
}

static String wifiMdnsUrl() {
  return String("http://") + wifiMdnsHost() + ".local";
}

static bool wifiPortalApActive() {
  return wifiApMode;
}

static String wifiPortalApSsid() {
  return wifiApSsid;
}

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

static String cleanVersion(String v) {
  v.trim();
  if (v.startsWith("v") || v.startsWith("V")) v.remove(0, 1);
  return v;
}

static int versionPart(String v, int index) {
  v = cleanVersion(v);
  int start = 0;
  for (int i = 0; i < index; i++) {
    int dot = v.indexOf('.', start);
    if (dot < 0) return 0;
    start = dot + 1;
  }
  int end = v.indexOf('.', start);
  if (end < 0) end = v.length();
  return v.substring(start, end).toInt();
}

static bool isNewVersion(String latest, String current) {
  latest = cleanVersion(latest);
  current = cleanVersion(current);
  for (int i = 0; i < 3; i++) {
    int l = versionPart(latest, i);
    int c = versionPart(current, i);
    if (l > c) return true;
    if (l < c) return false;
  }
  return false;
}

static String jsonStringValue(const String &json, const char *key, const String &fallback = String()) {
  String pattern = String("\"") + key + "\"";
  int pos = json.indexOf(pattern);
  if (pos < 0) return fallback;
  pos = json.indexOf(':', pos + pattern.length());
  if (pos < 0) return fallback;
  pos++;
  while (pos < (int)json.length() && isspace((unsigned char)json[pos])) pos++;
  if (pos >= (int)json.length() || json[pos] != '"') return fallback;
  pos++;

  String value;
  bool escaped = false;
  for (; pos < (int)json.length(); pos++) {
    char c = json[pos];
    if (escaped) {
      if (c == 'n') value += '\n';
      else if (c == 'r') value += '\r';
      else if (c == 't') value += '\t';
      else value += c;
      escaped = false;
    } else if (c == '\\') {
      escaped = true;
    } else if (c == '"') {
      return value;
    } else {
      value += c;
    }
  }
  return fallback;
}

struct ReleaseInfo {
  bool ok = false;
  String firmwareVersion;
  String webVersion;
  String firmwareUrl;
  String filesystemUrl;
  String error;
};

static void loadInstalledWebVersion() {
  Preferences prefs;
  if (!prefs.begin(WIFI_NVS_NAMESPACE, true)) {
    installedWebVersion = WEB_VERSION;
    return;
  }
  installedWebVersion = prefs.getString(OTA_WEB_VERSION_KEY, WEB_VERSION);
  prefs.end();
}

static bool saveInstalledWebVersion(const String &version) {
  Preferences prefs;
  if (!prefs.begin(WIFI_NVS_NAMESPACE, false)) {
    Serial.println("OTA web version save failed: NVS open failed.");
    return false;
  }
  prefs.putString(OTA_WEB_VERSION_KEY, version);
  prefs.end();
  installedWebVersion = version;
  return true;
}

static ReleaseInfo fetchReleaseInfo() {
  ReleaseInfo info;

  if (WiFi.status() != WL_CONNECTED) {
    info.error = "WiFi is not connected";
    return info;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  https.setTimeout(15000);
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  Serial.println("Checking GitHub release version...");
  Serial.print("Version URL: ");
  Serial.println(VERSION_URL);

  if (!https.begin(client, VERSION_URL)) {
    info.error = "HTTPS begin failed";
    return info;
  }

  int httpCode = https.GET();
  if (httpCode != HTTP_CODE_OK) {
    info.error = "HTTP code " + String(httpCode);
    https.end();
    return info;
  }

  String payload = https.getString();
  https.end();

  info.firmwareVersion = jsonStringValue(payload, "firmware_version");
  if (info.firmwareVersion.isEmpty()) {
    info.firmwareVersion = jsonStringValue(payload, "version");
  }
  info.webVersion = jsonStringValue(payload, "web_version", WEB_VERSION);
  info.firmwareUrl = jsonStringValue(payload, "firmware", FIRMWARE_URL_FALLBACK);
  info.filesystemUrl = jsonStringValue(payload, "filesystem", FILESYSTEM_URL_FALLBACK);

  info.firmwareVersion.trim();
  info.webVersion.trim();
  info.firmwareUrl.trim();
  info.filesystemUrl.trim();

  if (info.firmwareVersion.isEmpty()) {
    info.error = "firmware_version is empty";
    return info;
  }

  info.ok = true;
  return info;
}

static String otaVersionJson(const ReleaseInfo &info) {
  bool firmwareUpdate = info.ok && isNewVersion(info.firmwareVersion, FW_VERSION);
  bool webUpdate = info.ok && isNewVersion(info.webVersion, installedWebVersion);

  String json = F("{\"ok\":");
  json += info.ok ? F("true") : F("false");
  if (!info.ok) {
    json += F(",\"message\":\"");
    json += jsonEscape(info.error);
    json += F("\"}");
    return json;
  }
  json += F(",\"current_firmware\":\"");
  json += jsonEscape(FW_VERSION);
  json += F("\",\"latest_firmware\":\"");
  json += jsonEscape(info.firmwareVersion);
  json += F("\",\"firmware_update_available\":");
  json += firmwareUpdate ? F("true") : F("false");
  json += F(",\"current_web\":\"");
  json += jsonEscape(installedWebVersion);
  json += F("\",\"latest_web\":\"");
  json += jsonEscape(info.webVersion);
  json += F("\",\"web_update_available\":");
  json += webUpdate ? F("true") : F("false");
  json += F(",\"firmware_url\":\"");
  json += jsonEscape(info.firmwareUrl);
  json += F("\",\"filesystem_url\":\"");
  json += jsonEscape(info.filesystemUrl);
  json += F("\"}");
  return json;
}

static bool updateLittleFSFromURL(const String &url, const String &newWebVersion) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("LittleFS OTA skipped: WiFi is not connected.");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient https;
  https.setTimeout(30000);
  https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  Serial.println("Starting LittleFS OTA...");
  Serial.print("LittleFS URL: ");
  Serial.println(url);

  if (!https.begin(client, url)) {
    Serial.println("LittleFS OTA HTTPS begin failed.");
    return false;
  }

  int httpCode = https.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.print("LittleFS OTA HTTP error: ");
    Serial.println(httpCode);
    https.end();
    return false;
  }

  int contentLength = https.getSize();
  if (contentLength <= 0) {
    Serial.println("LittleFS OTA invalid content length.");
    https.end();
    return false;
  }

  WiFiClient *stream = https.getStreamPtr();
  LittleFS.end();

  if (!Update.begin(contentLength, U_SPIFFS)) {
    Serial.println("LittleFS OTA Update.begin failed.");
    Update.printError(Serial);
    https.end();
    LittleFS.begin(false);
    return false;
  }

  size_t written = Update.writeStream(*stream);
  Serial.print("LittleFS OTA written: ");
  Serial.print(written);
  Serial.print("/");
  Serial.println(contentLength);

  if (written != (size_t)contentLength) {
    Serial.println("LittleFS OTA written size mismatch.");
    Update.abort();
    https.end();
    LittleFS.begin(false);
    return false;
  }

  if (!Update.end() || !Update.isFinished()) {
    Serial.println("LittleFS OTA finish failed.");
    Update.printError(Serial);
    https.end();
    LittleFS.begin(false);
    return false;
  }

  https.end();
  saveInstalledWebVersion(newWebVersion);
  Serial.println("LittleFS OTA success.");
  return true;
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
  String host = wifiMdnsHost();
  WiFi.setHostname(host.c_str());
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
  Serial.print("Trying default WiFi first, attempts=");
  Serial.println(WIFI_DEFAULT_ATTEMPTS);
  for (uint8_t attempt = 1; attempt <= WIFI_DEFAULT_ATTEMPTS; attempt++) {
    Serial.print("Default WiFi attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(WIFI_DEFAULT_ATTEMPTS);
    if (connectWifiCredential(c, WIFI_DEFAULT_TIMEOUT_MS)) return true;
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  return false;
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
    s += F("<br>Local URL: <a href='");
    s += htmlEscape(wifiMdnsUrl());
    s += F("'>");
    s += htmlEscape(wifiMdnsUrl());
    s += F("</a>");
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
  json += F("\",\"hostname\":\"");
  json += jsonEscape(wifiMdnsHost());
  json += F("\",\"local_url\":\"");
  json += jsonEscape((WiFi.status() == WL_CONNECTED) ? wifiMdnsUrl() : String(""));
  json += F("\",\"rssi\":");
  if (WiFi.status() == WL_CONNECTED) {
    json += String(WiFi.RSSI());
  } else {
    json += F("null");
  }
  json += F(",\"ap_active\":");
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
  json += F("\",\"firmware_version\":\"");
  json += jsonEscape(FW_VERSION);
  json += F("\",\"web_version\":\"");
  json += jsonEscape(installedWebVersion);
  json += F("\",\"saved_count\":");
  json += String(wifiCredCount);
  json += F(",\"measurement_running\":");
  json += measurementEnabled ? F("true") : F("false");
  json += F(",\"measurement_busy\":");
  json += measurementBusy ? F("true") : F("false");
  json += F(",\"measurement_mode\":\"");
  json += measurementModeToText(getMeasurementMode());
  json += F("\"");
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

static String latestMeasurementJson() {
  String json = F("null");
  if (!hasLastMeasurement) return json;

  MeasurementData data = lastMeasurement;
  json = F("{\"mode\":\"");
  json += measurementModeToText(data.mode);
  json += F("\",\"direction\":\"");
  json += directionToText(data.direction);
  json += F("\",\"sample_index\":");
  json += String(data.sampleIndex);
  json += F(",\"dac_set_v\":");
  json += String(data.dacSetVoltage, 3);
  json += F(",\"potential_v\":");
  json += String(dacVoltageToPotential(data.dacSetVoltage), 3);
  json += F(",\"dac_code\":");
  json += String(data.dacCode);
  json += F(",\"dac_expected_v\":");
  json += String(data.dacExpectedVoltage, 6);
  json += F(",\"adc_raw\":");
  json += String(data.adcRaw);
  json += F(",\"adc_v\":");
  json += String(data.adcVoltage, 6);
  json += F(",\"adc_delta_v\":");
  json += String(data.adcDeltaVoltage, 6);
  json += F(",\"current_a\":");
  json += String(data.currentAmp, 12);
  json += F(",\"rtia_ohm\":");
  json += String(data.rtiaOhm, 0);
  json += F(",\"tia_cn\":\"0x");
  if (data.tiaCN < 0x10) json += '0';
  json += String(data.tiaCN, HEX);
  json += F("\",\"auto_range\":\"");
  json += data.autoRangeAction;
  json += F("\",\"timestamp_us\":");
  char timestampBuf[24];
  snprintf(timestampBuf, sizeof(timestampBuf), "%llu", (unsigned long long)data.timestamp_us);
  json += timestampBuf;
  json += F(",\"period_us\":");
  json += String(data.period_us);
  json += F("}");
  return json;
}

struct ContactPrecheckPoint {
  float potentialV;
  float dacV;
  uint16_t adcRaw;
  float adcV;
  float currentA;
};

static ContactPrecheckPoint readContactPrecheckPoint(float potentialV) {
  ContactPrecheckPoint point;
  point.potentialV = clampFloat(potentialV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  point.dacV = potentialToDacVoltage(point.potentialV);
  point.adcRaw = 0;
  point.adcV = 0.0f;
  point.currentA = 0.0f;

  writeAD5680_safe(voltageToDACCode(point.dacV));
  vTaskDelay(pdMS_TO_TICKS(CONTACT_PRECHECK_HOLD_MS));

  point.adcRaw = readAD7694Average_safe(CONTACT_PRECHECK_ADC_SAMPLES);
  point.adcV = adcRawToVoltage(point.adcRaw);
  point.currentA = (point.adcV - ADC_ZERO_VOLTAGE) / getCurrentRtiaOhm();
  return point;
}

static bool contactPrecheckPointNearRail(const ContactPrecheckPoint &point) {
  return point.adcV < CONTACT_PRECHECK_RAIL_MARGIN_V ||
         point.adcV > (ADC_VREF - CONTACT_PRECHECK_RAIL_MARGIN_V);
}

static void appendContactPrecheckPointJson(String &json, const ContactPrecheckPoint &point) {
  json += F("{\"e_v\":");
  json += String(point.potentialV, 3);
  json += F(",\"dac_v\":");
  json += String(point.dacV, 3);
  json += F(",\"adc_raw\":");
  json += String(point.adcRaw);
  json += F(",\"adc_v\":");
  json += String(point.adcV, 6);
  json += F(",\"current_a\":");
  json += String(point.currentA, 12);
  json += F("}");
}

static String runContactPrecheckJson() {
  uint32_t startedMs = millis();
  measurementBusy = true;

  int8_t savedRangeIndex = currentTiaRangeIndex;
  autoRangeLowCount = 0;
  autoRangeHighCount = 0;
  setTiaRangeIndex_safe(4); // 35k gives a stable, repeatable contact check range.

  ContactPrecheckPoint zero = readContactPrecheckPoint(0.0f);
  ContactPrecheckPoint positive = readContactPrecheckPoint(CONTACT_PRECHECK_PULSE_V);
  ContactPrecheckPoint negative = readContactPrecheckPoint(-CONTACT_PRECHECK_PULSE_V);
  float precheckRtiaOhm = getCurrentRtiaOhm();
  uint8_t precheckTiaCN = getCurrentTIACN();

  float responseAdcV = fabsf(positive.adcV - negative.adcV);
  float responseCurrentA = fabsf(positive.currentA - negative.currentA);
  float maxAbsCurrentA = fmaxf(fabsf(zero.currentA), fmaxf(fabsf(positive.currentA), fabsf(negative.currentA)));
  bool rail = contactPrecheckPointNearRail(zero) ||
              contactPrecheckPointNearRail(positive) ||
              contactPrecheckPointNearRail(negative);
  bool weak = responseAdcV < CONTACT_PRECHECK_MIN_RESPONSE_V &&
              maxAbsCurrentA < CONTACT_PRECHECK_MIN_CURRENT_A;
  bool contactOk = !rail && !weak;

  const char *status = contactOk ? "OK" : "WARN";
  const char *message = contactOk ? "Contact response detected" :
      (rail ? "ADC is near rail; check electrode contact, range, or wiring" :
              "Weak or no response; check WE/RE/CE contact and solution conductivity");

  writeAD5680_safe(voltageToDACCode(potentialToDacVoltage(0.0f)));
  if (savedRangeIndex >= 0 && savedRangeIndex < TIA_RANGE_COUNT && savedRangeIndex != currentTiaRangeIndex) {
    setTiaRangeIndex_safe(savedRangeIndex);
  }
  measurementBusy = false;

  String json = F("{\"ok\":true,\"contact_ok\":");
  json += contactOk ? F("true") : F("false");
  json += F(",\"status\":\"");
  json += status;
  json += F("\",\"message\":\"");
  json += jsonEscape(String(message));
  json += F("\",\"pulse_v\":");
  json += String(CONTACT_PRECHECK_PULSE_V, 3);
  json += F(",\"hold_ms\":");
  json += String(CONTACT_PRECHECK_HOLD_MS);
  json += F(",\"duration_ms\":");
  json += String(millis() - startedMs);
  json += F(",\"response_adc_v\":");
  json += String(responseAdcV, 6);
  json += F(",\"response_current_a\":");
  json += String(responseCurrentA, 12);
  json += F(",\"max_abs_current_a\":");
  json += String(maxAbsCurrentA, 12);
  json += F(",\"rtia_ohm\":");
  json += String(precheckRtiaOhm, 0);
  json += F(",\"tia_cn\":\"0x");
  if (precheckTiaCN < 0x10) json += '0';
  json += String(precheckTiaCN, HEX);
  json += F("\",\"zero\":");
  appendContactPrecheckPointJson(json, zero);
  json += F(",\"positive\":");
  appendContactPrecheckPointJson(json, positive);
  json += F(",\"negative\":");
  appendContactPrecheckPointJson(json, negative);
  json += F("}");
  return json;
}

static String cvConfigJson() {
  CvConfig cfg = cvConfig;
  String json = F("{\"experiment_name\":\"");
  json += jsonEscape(String(cfg.experimentName));
  json += F("\",\"start_v\":");
  json += String(cfg.startV, 4);
  json += F(",\"vertex1_v\":");
  json += String(cfg.vertex1V, 4);
  json += F(",\"vertex2_v\":");
  json += String(cfg.vertex2V, 4);
  json += F(",\"final_v\":");
  json += String(cfg.finalV, 4);
  json += F(",\"step_v\":");
  json += String(cfg.stepV, 4);
  json += F(",\"scan_rate_vps\":");
  json += String(cfg.scanRateVps, 4);
  json += F(",\"cycles\":");
  json += String(cfg.cycles);
  json += F(",\"quiet_ms\":");
  json += String(cfg.quietMs);
  json += F(",\"min_v\":");
  json += String(CV_MIN_POTENTIAL_V, 3);
  json += F(",\"max_v\":");
  json += String(CV_MAX_POTENTIAL_V, 3);
  json += F("}");
  return json;
}

static String swvConfigJson() {
  SwvConfig cfg = swvConfig;
  String json = F("{\"experiment_name\":\"");
  json += jsonEscape(String(cfg.experimentName));
  json += F("\",\"start_v\":");
  json += String(cfg.startV, 4);
  json += F(",\"end_v\":");
  json += String(cfg.endV, 4);
  json += F(",\"step_v\":");
  json += String(cfg.stepV, 4);
  json += F(",\"amplitude_v\":");
  json += String(cfg.amplitudeV, 4);
  json += F(",\"frequency_hz\":");
  json += String(cfg.frequencyHz, 4);
  json += F(",\"period_ms\":");
  json += String(cfg.periodMs, 4);
  json += F(",\"duty_ms\":");
  json += String(cfg.dutyMs, 4);
  json += F(",\"duty_percent\":");
  json += String(cfg.dutyPercent, 4);
  json += F(",\"timing_mode\":\"");
  json += cfg.useFrequency ? F("frequency") : F("period");
  json += F("\",\"duty_mode\":\"");
  json += cfg.useDutyPercent ? F("percent") : F("time");
  json += F("\"");
  json += F(",\"conditioning_enabled\":");
  json += cfg.conditioningEnabled ? F("true") : F("false");
  json += F(",\"conditioning_v\":");
  json += String(cfg.conditioningV, 4);
  json += F(",\"conditioning_ms\":");
  json += String(cfg.conditioningMs);
  json += F(",\"agitation_enabled\":");
  json += cfg.agitationEnabled ? F("true") : F("false");
  json += F(",\"agitation_on_ms\":");
  json += String(cfg.agitationOnMs);
  json += F(",\"agitation_off_ms\":");
  json += String(cfg.agitationOffMs);
  json += F(",\"motor_power_percent\":");
  json += String(cfg.motorPowerPercent);
  json += F(",\"settle_after_ms\":");
  json += String(cfg.settleAfterMs);
  json += F(",\"quiet_ms\":");
  json += String(cfg.quietMs);
  json += F(",\"min_v\":");
  json += String(CV_MIN_POTENTIAL_V, 3);
  json += F(",\"max_v\":");
  json += String(CV_MAX_POTENTIAL_V, 3);
  json += F("}");
  return json;
}

static String dpvConfigJson() {
  DpvConfig cfg = dpvConfig;
  String json = F("{\"experiment_name\":\"");
  json += jsonEscape(String(cfg.experimentName));
  json += F("\",\"start_v\":");
  json += String(cfg.startV, 4);
  json += F(",\"end_v\":");
  json += String(cfg.endV, 4);
  json += F(",\"step_v\":");
  json += String(cfg.stepV, 4);
  json += F(",\"amplitude_v\":");
  json += String(cfg.amplitudeV, 4);
  json += F(",\"period_ms\":");
  json += String(cfg.periodMs, 4);
  json += F(",\"pulse_ms\":");
  json += String(cfg.pulseMs, 4);
  json += F(",\"conditioning_enabled\":");
  json += cfg.conditioningEnabled ? F("true") : F("false");
  json += F(",\"conditioning_v\":");
  json += String(cfg.conditioningV, 4);
  json += F(",\"conditioning_ms\":");
  json += String(cfg.conditioningMs);
  json += F(",\"agitation_enabled\":");
  json += cfg.agitationEnabled ? F("true") : F("false");
  json += F(",\"agitation_on_ms\":");
  json += String(cfg.agitationOnMs);
  json += F(",\"agitation_off_ms\":");
  json += String(cfg.agitationOffMs);
  json += F(",\"motor_power_percent\":");
  json += String(cfg.motorPowerPercent);
  json += F(",\"settle_after_ms\":");
  json += String(cfg.settleAfterMs);
  json += F(",\"quiet_ms\":");
  json += String(cfg.quietMs);
  json += F(",\"min_v\":");
  json += String(CV_MIN_POTENTIAL_V, 3);
  json += F(",\"max_v\":");
  json += String(CV_MAX_POTENTIAL_V, 3);
  json += F("}");
  return json;
}

static String measurementStatusJson() {
  String json = F("{\"running\":");
  json += measurementEnabled ? F("true") : F("false");
  json += F(",\"busy\":");
  json += measurementBusy ? F("true") : F("false");
  json += F(",\"stop_requested\":");
  json += measurementStopRequested ? F("true") : F("false");
  json += F(",\"completed\":");
  json += measurementCompleted ? F("true") : F("false");
  json += F(",\"cycle_current\":");
  json += String(measurementCycleCurrent);
  json += F(",\"cycle_total\":");
  json += String(measurementCycleTotal);
  json += F(",\"progress_percent\":");
  json += String((float)measurementProgressPermille * 0.1f, 1);
  json += F(",\"mode\":\"");
  json += measurementModeToText(getMeasurementMode());
  json += F("\",\"modes\":[\"SWEEP\",\"HOLD_CENTER\",\"CV\",\"SWV\",\"DPV\"]");
  json += F(",\"cv_config\":");
  json += cvConfigJson();
  json += F(",\"swv_config\":");
  json += swvConfigJson();
  json += F(",\"dpv_config\":");
  json += dpvConfigJson();
  json += F(",\"sd_mounted\":");
  json += sdMounted ? F("true") : F("false");
  json += F(",\"sd_log_file_open\":");
  json += sdLogFileOpen ? F("true") : F("false");
  json += F(",\"cv_log_active\":");
  json += cvLogActive ? F("true") : F("false");
  json += F(",\"cv_file\":\"");
  json += jsonEscape(String(activeCvFilePath));
  json += F("\",\"sd_records_written\":");
  json += String(sdRecordWriteCount);
  json += F(",\"sd_records_dropped\":");
  json += String(sdRecordDropCount);
  json += F(",\"sd_write_errors\":");
  json += String(sdWriteErrorCount);
  json += F(",\"rtc_available\":");
  json += rtcAvailable ? F("true") : F("false");
  json += F(",\"rtc_utc_synced\":");
  json += rtcUtcSynced ? F("true") : F("false");
  json += F(",\"latest\":");
  json += latestMeasurementJson();
  json += F("}");
  return json;
}

static void handleMeasurementModeApi() {
  MeasurementMode requested = parseMeasurementMode(wifiServer.arg("mode"), getMeasurementMode());
  if (!setMeasurementMode(requested)) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing mode\"}"));
    return;
  }

  requestTftFullRefresh();
  wifiServer.send(200, "application/json", measurementStatusJson());
}

static float webArgFloat(const char *name, float fallback) {
  if (!wifiServer.hasArg(name)) return fallback;
  String value = wifiServer.arg(name);
  value.trim();
  if (value.length() == 0) return fallback;
  return value.toFloat();
}

static uint32_t webArgUInt(const char *name, uint32_t fallback) {
  if (!wifiServer.hasArg(name)) return fallback;
  String value = wifiServer.arg(name);
  value.trim();
  if (value.length() == 0) return fallback;
  return (uint32_t)value.toInt();
}

static bool webArgBool(const char *name, bool fallback) {
  if (!wifiServer.hasArg(name)) return fallback;
  String value = wifiServer.arg(name);
  value.trim();
  value.toLowerCase();
  return value == "1" || value == "true" || value == "on" || value == "yes";
}

static bool applyCvConfigFromRequest() {
  if (measurementEnabled || measurementBusy) {
    return false;
  }

  CvConfig next = cvConfig;
  if (wifiServer.hasArg("experiment_name")) {
    sanitizeExperimentName(wifiServer.arg("experiment_name"), next.experimentName, sizeof(next.experimentName));
  }
  next.startV = webArgFloat("start_v", next.startV);
  next.vertex1V = webArgFloat("vertex1_v", next.vertex1V);
  next.vertex2V = webArgFloat("vertex2_v", next.vertex2V);
  next.finalV = webArgFloat("final_v", next.finalV);
  next.stepV = webArgFloat("step_v", next.stepV);
  next.scanRateVps = webArgFloat("scan_rate_vps", next.scanRateVps);
  next.cycles = (uint16_t)webArgUInt("cycles", next.cycles);
  next.quietMs = webArgUInt("quiet_ms", next.quietMs);

  next.startV = clampFloat(next.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.vertex1V = clampFloat(next.vertex1V, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.vertex2V = clampFloat(next.vertex2V, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.finalV = clampFloat(next.finalV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.stepV = clampFloat(fabsf(next.stepV), 0.0005f, 0.25f);
  next.scanRateVps = clampFloat(fabsf(next.scanRateVps), 0.001f, 10.0f);
  if (next.cycles == 0) next.cycles = 1;
  if (next.cycles > 100) next.cycles = 100;
  if (next.quietMs > 600000UL) next.quietMs = 600000UL;

  cvConfig = next;
  return true;
}

static bool applySwvConfigFromRequest() {
  if (measurementEnabled || measurementBusy) {
    return false;
  }

  SwvConfig next = swvConfig;
  if (wifiServer.hasArg("experiment_name")) {
    sanitizeExperimentName(wifiServer.arg("experiment_name"), next.experimentName, sizeof(next.experimentName));
  }
  next.startV = webArgFloat("start_v", next.startV);
  next.endV = webArgFloat("end_v", next.endV);
  next.stepV = webArgFloat("step_v", next.stepV);
  next.amplitudeV = webArgFloat("amplitude_v", next.amplitudeV);
  next.frequencyHz = webArgFloat("frequency_hz", next.frequencyHz);
  next.periodMs = webArgFloat("period_ms", next.periodMs);
  next.dutyMs = webArgFloat("duty_ms", next.dutyMs);
  next.dutyPercent = webArgFloat("duty_percent", next.dutyPercent);
  if (wifiServer.hasArg("timing_mode")) {
    String mode = wifiServer.arg("timing_mode");
    mode.trim();
    mode.toLowerCase();
    next.useFrequency = (mode == "frequency" || mode == "freq");
  }
  if (wifiServer.hasArg("duty_mode")) {
    String mode = wifiServer.arg("duty_mode");
    mode.trim();
    mode.toLowerCase();
    next.useDutyPercent = (mode == "percent" || mode == "percentage");
  }
  next.conditioningEnabled = webArgBool("conditioning_enabled", false);
  next.conditioningV = webArgFloat("conditioning_v", next.conditioningV);
  next.conditioningMs = webArgUInt("conditioning_ms", next.conditioningMs);
  next.agitationEnabled = webArgBool("agitation_enabled", false);
  next.agitationOnMs = webArgUInt("agitation_on_ms", next.agitationOnMs);
  next.agitationOffMs = webArgUInt("agitation_off_ms", next.agitationOffMs);
  next.motorPowerPercent = (uint8_t)webArgUInt("motor_power_percent", next.motorPowerPercent);
  next.settleAfterMs = webArgUInt("settle_after_ms", next.settleAfterMs);
  next.quietMs = webArgUInt("quiet_ms", next.quietMs);

  next.startV = clampFloat(next.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.endV = clampFloat(next.endV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.stepV = clampFloat(fabsf(next.stepV), 0.0005f, 0.25f);
  next.amplitudeV = clampFloat(fabsf(next.amplitudeV), 0.0005f, 1.0f);
  next.frequencyHz = clampFloat(fabsf(next.frequencyHz), 0.1f, 500.0f);
  next.periodMs = clampFloat(fabsf(next.periodMs), 2.0f, 10000.0f);
  next.frequencyHz = next.useFrequency ? next.frequencyHz : (1000.0f / next.periodMs);
  next.periodMs = 1000.0f / next.frequencyHz;
  next.dutyPercent = clampFloat(fabsf(next.dutyPercent), 1.0f, 99.0f);
  next.dutyMs = fabsf(next.dutyMs);
  if (next.useDutyPercent) {
    next.dutyMs = next.periodMs * next.dutyPercent * 0.01f;
  } else {
    next.dutyMs = clampFloat(next.dutyMs, 0.1f, next.periodMs - 0.1f);
    next.dutyPercent = (next.dutyMs / next.periodMs) * 100.0f;
  }
  next.conditioningV = clampFloat(next.conditioningV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  if (next.conditioningMs > 600000UL) next.conditioningMs = 600000UL;
  if (next.agitationOnMs < 100UL) next.agitationOnMs = 100UL;
  if (next.agitationOffMs < 100UL) next.agitationOffMs = 100UL;
  if (next.motorPowerPercent > 100) next.motorPowerPercent = 100;
  if (next.settleAfterMs > 60000UL) next.settleAfterMs = 60000UL;
  if (next.quietMs > 600000UL) next.quietMs = 600000UL;

  swvConfig = next;
  return true;
}

static bool applyDpvConfigFromRequest() {
  if (measurementEnabled || measurementBusy) {
    return false;
  }

  DpvConfig next = dpvConfig;
  if (wifiServer.hasArg("experiment_name")) {
    sanitizeExperimentName(wifiServer.arg("experiment_name"), next.experimentName, sizeof(next.experimentName));
  }
  next.startV = webArgFloat("start_v", next.startV);
  next.endV = webArgFloat("end_v", next.endV);
  next.stepV = webArgFloat("step_v", next.stepV);
  next.amplitudeV = webArgFloat("amplitude_v", next.amplitudeV);
  next.periodMs = webArgFloat("period_ms", next.periodMs);
  next.pulseMs = webArgFloat("pulse_ms", next.pulseMs);
  next.conditioningEnabled = webArgBool("conditioning_enabled", false);
  next.conditioningV = webArgFloat("conditioning_v", next.conditioningV);
  next.conditioningMs = webArgUInt("conditioning_ms", next.conditioningMs);
  next.agitationEnabled = webArgBool("agitation_enabled", false);
  next.agitationOnMs = webArgUInt("agitation_on_ms", next.agitationOnMs);
  next.agitationOffMs = webArgUInt("agitation_off_ms", next.agitationOffMs);
  next.motorPowerPercent = (uint8_t)webArgUInt("motor_power_percent", next.motorPowerPercent);
  next.settleAfterMs = webArgUInt("settle_after_ms", next.settleAfterMs);
  next.quietMs = webArgUInt("quiet_ms", next.quietMs);

  next.startV = clampFloat(next.startV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.endV = clampFloat(next.endV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  next.stepV = clampFloat(fabsf(next.stepV), 0.0005f, 0.25f);
  next.amplitudeV = clampFloat(fabsf(next.amplitudeV), 0.0005f, 1.0f);
  next.periodMs = clampFloat(fabsf(next.periodMs), 2.0f, 10000.0f);
  next.pulseMs = clampFloat(fabsf(next.pulseMs), 0.1f, next.periodMs - 0.1f);
  next.conditioningV = clampFloat(next.conditioningV, CV_MIN_POTENTIAL_V, CV_MAX_POTENTIAL_V);
  if (next.conditioningMs > 600000UL) next.conditioningMs = 600000UL;
  if (next.agitationOnMs < 100UL) next.agitationOnMs = 100UL;
  if (next.agitationOffMs < 100UL) next.agitationOffMs = 100UL;
  if (next.motorPowerPercent > 100) next.motorPowerPercent = 100;
  if (next.settleAfterMs > 60000UL) next.settleAfterMs = 60000UL;
  if (next.quietMs > 600000UL) next.quietMs = 600000UL;

  dpvConfig = next;
  return true;
}

static void handleCvConfigApi() {
  if (!applyCvConfigFromRequest()) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing CV parameters\"}"));
    return;
  }

  currentMeasurementMode = MEAS_MODE_CV;
  wifiServer.send(200, "application/json", measurementStatusJson());
}

static void handleSwvConfigApi() {
  if (!applySwvConfigFromRequest()) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing SWV parameters\"}"));
    return;
  }

  currentMeasurementMode = MEAS_MODE_SWV;
  wifiServer.send(200, "application/json", measurementStatusJson());
}

static void handleDpvConfigApi() {
  if (!applyDpvConfigFromRequest()) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing DPV parameters\"}"));
    return;
  }

  currentMeasurementMode = MEAS_MODE_DPV;
  wifiServer.send(200, "application/json", measurementStatusJson());
}

static void handleMeasurementStartApi() {
  bool hasDpvArgs = wifiServer.hasArg("pulse_ms") || wifiServer.arg("mode") == "DPV";
  bool hasSwvArgs = wifiServer.hasArg("end_v") || wifiServer.hasArg("amplitude_v") ||
      wifiServer.hasArg("frequency_hz") || wifiServer.hasArg("period_ms") ||
      wifiServer.hasArg("duty_ms") || wifiServer.hasArg("duty_percent") ||
      wifiServer.hasArg("conditioning_enabled") || wifiServer.hasArg("conditioning_v") ||
      wifiServer.hasArg("conditioning_ms") || wifiServer.hasArg("agitation_enabled");
  bool hasCvArgs = wifiServer.hasArg("start_v") || wifiServer.hasArg("vertex1_v") ||
      wifiServer.hasArg("vertex2_v") || wifiServer.hasArg("final_v") ||
      wifiServer.hasArg("step_v") || wifiServer.hasArg("scan_rate_vps") ||
      wifiServer.hasArg("cycles") || wifiServer.hasArg("quiet_ms") ||
      wifiServer.hasArg("experiment_name");

  if (hasDpvArgs) {
    if (!applyDpvConfigFromRequest()) {
      wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing DPV parameters\"}"));
      return;
    }
  } else if (hasSwvArgs) {
    if (!applySwvConfigFromRequest()) {
      wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing SWV parameters\"}"));
      return;
    }
  } else if (hasCvArgs) {
    if (!applyCvConfigFromRequest()) {
      wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing CV parameters\"}"));
      return;
    }
  }

  if (wifiServer.hasArg("mode")) {
    MeasurementMode requested = parseMeasurementMode(wifiServer.arg("mode"), getMeasurementMode());
    if (!setMeasurementMode(requested)) {
      wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before changing mode\"}"));
      return;
    }
  }

  if ((getMeasurementMode() == MEAS_MODE_CV || getMeasurementMode() == MEAS_MODE_SWV || getMeasurementMode() == MEAS_MODE_DPV) && WiFi.status() == WL_CONNECTED && !rtcUtcSynced) {
    syncUtcTimeFromNtpToRtc(5000);
  }

  if ((getMeasurementMode() == MEAS_MODE_CV || getMeasurementMode() == MEAS_MODE_SWV || getMeasurementMode() == MEAS_MODE_DPV) && !initSdCard()) {
    wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD card mount failed; measurement logging requires SD\"}"));
    return;
  }

  if (!measurementEnabled) {
    resetAutoRangeToLowestForNewRun();
    requestMeasurementRun();
    digitalWrite(LED_BUILTIN_PIN, LOW);
    refreshRunButtonFromOptionTask();
  }

  wifiServer.send(200, "application/json", measurementStatusJson());
}

static void handleMeasurementStopApi() {
  requestMeasurementStop();
  digitalWrite(LED_BUILTIN_PIN, HIGH);
  refreshRunButtonFromOptionTask();
  wifiServer.send(200, "application/json", measurementStatusJson());
}

static void handleContactPrecheckApi() {
  if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before contact pre-check\"}"));
    return;
  }

  wifiServer.send(200, "application/json", runContactPrecheckJson());
}

static void handleVibrationTestApi() {
  if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before testing vibration\"}"));
    return;
  }

  if (vibrationTestActive) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Vibration test is already running\"}"));
    return;
  }

  uint8_t powerPercent = (uint8_t)webArgUInt("motor_power_percent", 60);
  uint32_t durationMs = webArgUInt("duration_ms", 1000);
  if (powerPercent > 100) powerPercent = 100;
  if (durationMs < 100UL) durationMs = 100UL;
  if (durationMs > 3000UL) durationMs = 3000UL;

  vibrationTestActive = true;
  vibrationPulse(motorPercentToDuty(powerPercent), durationMs);
  vibrationTestActive = false;

  wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Vibration test completed\"}"));
}

static bool isSafeCvFilePath(const String &path) {
  return (path.startsWith("/cv_") || path.startsWith("/swv_") || path.startsWith("/dpv_")) && path.endsWith(".bin") && path.indexOf("..") < 0;
}

static String cvFilesJson() {
  String json = F("{\"files\":[");
#if ENABLE_SD_LOGGING
  if (!sdMounted && !measurementEnabled && !measurementBusy) {
    initSdCard();
  }
  if (sdMounted) {
    File root = SD_MMC.open("/");
    bool first = true;
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String name = file.name();
        if (!name.startsWith("/")) name = "/" + name;
        if (!file.isDirectory() && isSafeCvFilePath(name)) {
          if (!first) json += ',';
          first = false;
          json += F("{\"name\":\"");
          json += jsonEscape(name);
          json += F("\",\"size\":");
          json += String((uint32_t)file.size());
          json += F(",\"modified\":");
          json += String((uint32_t)file.getLastWrite());
          json += F("}");
        }
        file = root.openNextFile();
      }
      root.close();
    }
  }
#endif
  json += F("]}");
  return json;
}

static void handleCvFilesApi() {
  wifiServer.send(200, "application/json", cvFilesJson());
}

static void handleCvDownloadCsvApi() {
#if ENABLE_SD_LOGGING
  if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before downloading CV data\"}"));
    return;
  }

  if (!sdMounted) {
    if (!initSdCard()) {
      wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD card is not mounted\"}"));
      return;
    }
  }

  String path = wifiServer.arg("file");
  if (!isSafeCvFilePath(path)) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"Invalid CV file path\"}"));
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    wifiServer.send(404, "application/json", F("{\"ok\":false,\"message\":\"CV file not found\"}"));
    return;
  }

  CvBinHeader header;
  if (file.read((uint8_t *)&header, sizeof(header)) != sizeof(header) ||
      (memcmp(header.magic, "POTCVB1", 7) != 0 && memcmp(header.magic, "POTSWV1", 7) != 0 && memcmp(header.magic, "POTDPV1", 7) != 0) ||
      header.recordSize != sizeof(CvBinRecord)) {
    file.close();
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"Invalid CV binary file\"}"));
    return;
  }

  String downloadName = path.substring(1);
  downloadName.replace(".bin", ".csv");
  wifiServer.sendHeader("Content-Disposition", "attachment; filename=\"" + downloadName + "\"");
  wifiServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wifiServer.send(200, "text/csv", "");

  String modeName = F("CV");
  if (memcmp(header.magic, "POTDPV1", 7) == 0) {
    modeName = F("DPV");
    wifiServer.sendContent(F("mode,experiment_name,start_v,end_v,step_v,amplitude_v,period_ms,pulse_ms,quiet_ms\n"));
    wifiServer.sendContent(modeName + "," + String(header.experimentName) + "," +
                           String(header.startV, 6) + "," +
                           String(header.vertex1V, 6) + "," +
                           String(header.stepV, 6) + "," +
                           String(header.vertex2V, 6) + "," +
                           String(header.scanRateVps, 6) + "," +
                           String(header.finalV, 6) + "," +
                           String(header.quietMs) + "\n\n");
  } else if (memcmp(header.magic, "POTSWV1", 7) == 0) {
    modeName = F("SWV");
    float periodMs = 1000.0f / header.scanRateVps;
    float dutyMs = periodMs * header.finalV * 0.01f;
    wifiServer.sendContent(F("mode,experiment_name,start_v,end_v,step_v,amplitude_v,frequency_hz,period_ms,duty_ms,duty_percent,quiet_ms\n"));
    wifiServer.sendContent(modeName + "," + String(header.experimentName) + "," +
                           String(header.startV, 6) + "," +
                           String(header.vertex1V, 6) + "," +
                           String(header.stepV, 6) + "," +
                           String(header.vertex2V, 6) + "," +
                           String(header.scanRateVps, 6) + "," +
                           String(periodMs, 6) + "," +
                           String(dutyMs, 6) + "," +
                           String(header.finalV, 6) + "," +
                           String(header.quietMs) + "\n\n");
  } else {
    wifiServer.sendContent(F("mode,experiment_name,start_v,vertex1_v,vertex2_v,final_v,step_v,scan_rate_vps,cycles,quiet_ms\n"));
    wifiServer.sendContent(modeName + "," + String(header.experimentName) + "," +
                           String(header.startV, 6) + "," +
                           String(header.vertex1V, 6) + "," + String(header.vertex2V, 6) + "," +
                           String(header.finalV, 6) + "," + String(header.stepV, 6) + "," +
                           String(header.scanRateVps, 6) + "," + String(header.cycles) + "," +
                           String(header.quietMs) + "\n\n");
  }
  wifiServer.sendContent(F("sample_index,period_us,potential_v,current_uA\n"));

  CvBinRecord record;
  char line[128];
  bool firstRecord = true;
  uint64_t firstTimestampUs = 0;
  uint32_t outputSampleIndex = 0;
  while (file.read((uint8_t *)&record, sizeof(record)) == sizeof(record)) {
    if (firstRecord) {
      firstRecord = false;
      firstTimestampUs = record.timestampUs;
      outputSampleIndex = 0;
    }
    uint64_t elapsedUs64 = (record.timestampUs >= firstTimestampUs) ? (record.timestampUs - firstTimestampUs) : 0ULL;
    uint32_t elapsedUs = (elapsedUs64 > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)elapsedUs64;
    snprintf(
      line,
      sizeof(line),
      "%lu,%lu,%.6f,%.6f\n",
      (unsigned long)outputSampleIndex++,
      (unsigned long)elapsedUs,
      record.potentialV,
      record.currentA * 1000000.0f
    );
    wifiServer.sendContent(line);
    vTaskDelay(1);
  }

  file.close();
  wifiServer.sendContent("");
#else
  wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD logging disabled\"}"));
#endif
}

static void handleCvDownloadBinApi() {
#if ENABLE_SD_LOGGING
  if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before downloading CV data\"}"));
    return;
  }

  if (!sdMounted) {
    if (!initSdCard()) {
      wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD card is not mounted\"}"));
      return;
    }
  }

  String path = wifiServer.arg("file");
  if (!isSafeCvFilePath(path)) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"Invalid CV file path\"}"));
    return;
  }

  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    wifiServer.send(404, "application/json", F("{\"ok\":false,\"message\":\"CV file not found\"}"));
    return;
  }

  String downloadName = path.substring(1);
  wifiServer.sendHeader("Content-Disposition", "attachment; filename=\"" + downloadName + "\"");
  wifiServer.streamFile(file, "application/octet-stream");
  file.close();
#else
  wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD logging disabled\"}"));
#endif
}

static void handleCvDeleteApi() {
#if ENABLE_SD_LOGGING
  if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before deleting CV data\"}"));
    return;
  }

  if (!sdMounted) {
    if (!initSdCard()) {
      wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD card is not mounted\"}"));
      return;
    }
  }

  String path = wifiServer.arg("file");
  if (!isSafeCvFilePath(path)) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"Invalid CV file path\"}"));
    return;
  }

  if (!SD_MMC.exists(path)) {
    wifiServer.send(404, "application/json", F("{\"ok\":false,\"message\":\"CV file not found\"}"));
    return;
  }

  if (!SD_MMC.remove(path)) {
    wifiServer.send(500, "application/json", F("{\"ok\":false,\"message\":\"Delete failed\"}"));
    return;
  }

  wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"CV file deleted\"}"));
#else
  wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD logging disabled\"}"));
#endif
}

static void handlePlotlyJs() {
  wifiServer.sendHeader("Cache-Control", "public, max-age=86400");

#if ENABLE_SD_LOGGING
  bool sdAvailable = sdMounted;
  if (!sdAvailable && !measurementEnabled && !measurementBusy && !cvLogActive && !sdLogFileOpen) {
    sdAvailable = initSdCard();
  }

  if (sdAvailable && !measurementEnabled && !measurementBusy && !cvLogActive && !sdLogFileOpen && SD_MMC.exists(WIFI_PLOTLY_FILE)) {
    File f = SD_MMC.open(WIFI_PLOTLY_FILE, FILE_READ);
    if (f) {
      if (f.size() > 0) {
        wifiServer.streamFile(f, "application/javascript");
        f.close();
        return;
      }
      f.close();
    }
  }
  if (sdAvailable && !measurementEnabled && !measurementBusy && !cvLogActive && !sdLogFileOpen && SD_MMC.exists(WIFI_PLOTLY_LEGACY_FILE)) {
    File f = SD_MMC.open(WIFI_PLOTLY_LEGACY_FILE, FILE_READ);
    if (f) {
      wifiServer.streamFile(f, "application/javascript");
      f.close();
      return;
    }
  }
#endif

  if (LittleFS.exists(WIFI_PLOTLY_LEGACY_FILE)) {
    File f = LittleFS.open(WIFI_PLOTLY_LEGACY_FILE, FILE_READ);
    if (f) {
      wifiServer.streamFile(f, "application/javascript");
      f.close();
      return;
    }
  }

  wifiServer.send(404, "text/plain", "plotly.min.js not found");
}

static void handleBrandLogoPng() {
  if (!LittleFS.exists(BRAND_LOGO_FILE)) {
    wifiServer.send(404, "text/plain", "brand_logo.png not found");
    return;
  }

  File f = LittleFS.open(BRAND_LOGO_FILE, FILE_READ);
  if (!f) {
    wifiServer.send(404, "text/plain", "brand_logo.png not found");
    return;
  }

  wifiServer.sendHeader("Cache-Control", "public, max-age=86400");
  wifiServer.streamFile(f, "image/png");
  f.close();
}

static String sdAssetStatusJson() {
  String json = F("{\"sd_mounted\":");
  json += sdMounted ? F("true") : F("false");
  json += F(",\"asset_dir\":\"");
  json += WIFI_ASSET_DIR;
  json += F("\",\"plotly_path\":\"");
  json += WIFI_PLOTLY_FILE;
  json += F("\",\"plotly_exists\":");

#if ENABLE_SD_LOGGING
  bool sdAvailable = sdMounted;
  if (!sdAvailable && !measurementEnabled && !measurementBusy && !cvLogActive && !sdLogFileOpen) {
    sdAvailable = initSdCard();
  }
  bool exists = sdAvailable && SD_MMC.exists(WIFI_PLOTLY_FILE);
  json += exists ? F("true") : F("false");
  json += F(",\"plotly_size\":");
  if (exists) {
    File f = SD_MMC.open(WIFI_PLOTLY_FILE, FILE_READ);
    if (f) {
      json += String((uint32_t)f.size());
      f.close();
    } else {
      json += F("null");
    }
  } else {
    json += F("0");
  }
#else
  json += F("false,\"plotly_size\":0");
#endif

  json += F("}");
  return json;
}

static void handlePlotlyUploadResult() {
#if ENABLE_SD_LOGGING
  if (wifiAssetUploadOk) {
    String json = F("{\"ok\":true,\"message\":\"Uploaded to /device_assets/plotly.min.js\",\"bytes\":");
    json += String((uint32_t)wifiAssetUploadBytes);
    json += F("}");
    wifiServer.send(200, "application/json", json);
  } else {
    String json = F("{\"ok\":false,\"message\":\"");
    json += jsonEscape(wifiAssetUploadMessage.length() ? wifiAssetUploadMessage : String("Upload failed"));
    json += F("\"}");
    wifiServer.send(500, "application/json", json);
  }
#else
  wifiServer.send(503, "application/json", F("{\"ok\":false,\"message\":\"SD logging disabled\"}"));
#endif
}

static void handlePlotlyUploadStream() {
#if ENABLE_SD_LOGGING
  HTTPUpload &upload = wifiServer.upload();

  if (upload.status == UPLOAD_FILE_START) {
    wifiAssetUploadOk = false;
    wifiAssetUploadBytes = 0;
    wifiAssetUploadMessage = "";

    if (measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen) {
      wifiAssetUploadMessage = "Stop measurement before uploading SD assets";
      return;
    }

    if (!sdMounted && !initSdCard()) {
      wifiAssetUploadMessage = "SD card is not mounted";
      return;
    }

    if (!SD_MMC.exists(WIFI_ASSET_DIR)) {
      SD_MMC.mkdir(WIFI_ASSET_DIR);
    }

    if (SD_MMC.exists(WIFI_PLOTLY_TEMP_FILE)) {
      SD_MMC.remove(WIFI_PLOTLY_TEMP_FILE);
    }

    wifiAssetUploadFile = SD_MMC.open(WIFI_PLOTLY_TEMP_FILE, FILE_WRITE);
    if (!wifiAssetUploadFile) {
      wifiAssetUploadMessage = "Cannot open SD temp file";
      return;
    }

    wifiAssetUploadOk = true;
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (wifiAssetUploadOk && wifiAssetUploadFile) {
      size_t written = wifiAssetUploadFile.write(upload.buf, upload.currentSize);
      wifiAssetUploadBytes += written;
      if (written != upload.currentSize) {
        wifiAssetUploadOk = false;
        wifiAssetUploadMessage = "SD write failed";
      }
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (wifiAssetUploadFile) {
      wifiAssetUploadFile.close();
    }

    if (wifiAssetUploadOk) {
      if (SD_MMC.exists(WIFI_PLOTLY_FILE)) {
        SD_MMC.remove(WIFI_PLOTLY_FILE);
      }
      if (!SD_MMC.rename(WIFI_PLOTLY_TEMP_FILE, WIFI_PLOTLY_FILE)) {
        wifiAssetUploadOk = false;
        wifiAssetUploadMessage = "Cannot move temp file into /device_assets";
      } else {
        File check = SD_MMC.open(WIFI_PLOTLY_FILE, FILE_READ);
        if (!check || check.size() == 0) {
          wifiAssetUploadOk = false;
          wifiAssetUploadMessage = "Uploaded file is empty";
        }
        if (check) check.close();
      }
    }

    if (!wifiAssetUploadOk && SD_MMC.exists(WIFI_PLOTLY_TEMP_FILE)) {
      SD_MMC.remove(WIFI_PLOTLY_TEMP_FILE);
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (wifiAssetUploadFile) {
      wifiAssetUploadFile.close();
    }
    if (SD_MMC.exists(WIFI_PLOTLY_TEMP_FILE)) {
      SD_MMC.remove(WIFI_PLOTLY_TEMP_FILE);
    }
    wifiAssetUploadOk = false;
    wifiAssetUploadMessage = "Upload aborted";
  }
#endif
}

static bool otaBusyBlocked() {
  return measurementEnabled || measurementBusy || cvLogActive || sdLogFileOpen;
}

static void handleOtaVersionApi() {
  ReleaseInfo info = fetchReleaseInfo();
  wifiServer.send(info.ok ? 200 : 500, "application/json", otaVersionJson(info));
}

static void handleOtaFirmwareApi() {
  if (otaBusyBlocked()) {
    showOtaStatusScreen("Firmware OTA", "Measurement is running.", "Stop measurement before update.", TFT_RED);
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before OTA update\"}"));
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    showOtaStatusScreen("Firmware OTA", "WiFi is not connected.", "Connect WiFi before update.", TFT_RED);
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"WiFi is not connected\"}"));
    return;
  }

  showOtaStatusScreen("Firmware OTA", "Checking GitHub Release...", String("Current FW ") + FW_VERSION, TFT_CYAN);
  ReleaseInfo info = fetchReleaseInfo();
  if (!info.ok) {
    showOtaStatusScreen("Firmware OTA", "Version check failed.", info.error, TFT_RED);
    String json = F("{\"ok\":false,\"message\":\"");
    json += jsonEscape(info.error);
    json += F("\"}");
    wifiServer.send(500, "application/json", json);
    return;
  }

  if (!isNewVersion(info.firmwareVersion, FW_VERSION)) {
    showOtaStatusScreen("Firmware OTA", "Firmware is already current.", String("FW ") + FW_VERSION, TFT_GREEN);
    String json = F("{\"ok\":true,\"message\":\"Firmware is already current\",\"current_firmware\":\"");
    json += jsonEscape(FW_VERSION);
    json += F("\",\"latest_firmware\":\"");
    json += jsonEscape(info.firmwareVersion);
    json += F("\"}");
    wifiServer.send(200, "application/json", json);
    return;
  }

  String json = F("{\"ok\":true,\"message\":\"Starting firmware OTA. Device will restart if update succeeds.\",\"latest_firmware\":\"");
  json += jsonEscape(info.firmwareVersion);
  json += F("\"}");
  wifiServer.send(200, "application/json", json);
  delay(700);

  showOtaStatusScreen("Firmware OTA", String("Downloading FW ") + info.firmwareVersion, "Device will restart if successful.", TFT_YELLOW);
  Serial.println("Starting firmware OTA...");
  Serial.print("Firmware URL: ");
  Serial.println(info.firmwareUrl);

  WiFiClientSecure client;
  client.setInsecure();

  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);

  t_httpUpdate_return ret = httpUpdate.update(client, info.firmwareUrl);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      showOtaStatusScreen("Firmware OTA", "Update failed.", httpUpdate.getLastErrorString(), TFT_RED);
      Serial.printf("Firmware OTA failed. Error (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      showOtaStatusScreen("Firmware OTA", "No update available.", String("FW ") + FW_VERSION, TFT_GREEN);
      Serial.println("Firmware OTA: no updates.");
      break;
    case HTTP_UPDATE_OK:
      showOtaStatusScreen("Firmware OTA", "Update complete.", "Restarting...", TFT_GREEN);
      Serial.println("Firmware OTA OK. Rebooting...");
      break;
  }
}

static void handleOtaWebApi() {
  if (otaBusyBlocked()) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before web OTA update\"}"));
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"WiFi is not connected\"}"));
    return;
  }

  ReleaseInfo info = fetchReleaseInfo();
  if (!info.ok) {
    String json = F("{\"ok\":false,\"message\":\"");
    json += jsonEscape(info.error);
    json += F("\"}");
    wifiServer.send(500, "application/json", json);
    return;
  }

  if (!isNewVersion(info.webVersion, installedWebVersion)) {
    String json = F("{\"ok\":true,\"message\":\"Web app is already current\",\"current_web\":\"");
    json += jsonEscape(installedWebVersion);
    json += F("\",\"latest_web\":\"");
    json += jsonEscape(info.webVersion);
    json += F("\"}");
    wifiServer.send(200, "application/json", json);
    return;
  }

  wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Starting web app OTA. Device will restart if update succeeds.\"}"));
  delay(700);

  bool ok = updateLittleFSFromURL(info.filesystemUrl, info.webVersion);
  if (ok) {
    delay(1000);
    ESP.restart();
  }

  Serial.println("Web app OTA failed.");
}

static void handleOtaAllApi() {
  if (otaBusyBlocked()) {
    wifiServer.send(409, "application/json", F("{\"ok\":false,\"message\":\"Stop measurement before OTA update\"}"));
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    wifiServer.send(400, "application/json", F("{\"ok\":false,\"message\":\"WiFi is not connected\"}"));
    return;
  }

  ReleaseInfo info = fetchReleaseInfo();
  if (!info.ok) {
    String json = F("{\"ok\":false,\"message\":\"");
    json += jsonEscape(info.error);
    json += F("\"}");
    wifiServer.send(500, "application/json", json);
    return;
  }

  bool firmwareUpdate = isNewVersion(info.firmwareVersion, FW_VERSION);
  bool webUpdate = isNewVersion(info.webVersion, installedWebVersion);
  if (!firmwareUpdate && !webUpdate) {
    wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Firmware and web app are already current\"}"));
    return;
  }

  if (webUpdate) {
    wifiServer.send(200, "application/json", F("{\"ok\":true,\"message\":\"Starting web app OTA first. After restart, check version again for firmware.\"}"));
    delay(700);
    bool ok = updateLittleFSFromURL(info.filesystemUrl, info.webVersion);
    if (ok) {
      delay(1000);
      ESP.restart();
    }
    Serial.println("Web app OTA failed during update-all.");
    return;
  }

  handleOtaFirmwareApi();
}

static void startWifiWebServer() {
  if (wifiServerStarted) return;

  wifiServer.on("/", HTTP_GET, []() { sendWifiPortalIndex(); });
  wifiServer.on("/wifi_index.html", HTTP_GET, []() { sendWifiPortalIndex(); });
  wifiServer.on("/plotly.min.js", HTTP_GET, []() { handlePlotlyJs(); });
  wifiServer.on("/brand_logo.png", HTTP_GET, []() { handleBrandLogoPng(); });
  wifiServer.on("/generate_204", HTTP_GET, []() { wifiServer.sendHeader("Location", "/", true); wifiServer.send(302, "text/plain", ""); });
  wifiServer.on("/fwlink", HTTP_GET, []() { wifiServer.sendHeader("Location", "/", true); wifiServer.send(302, "text/plain", ""); });

  wifiServer.on("/api/status", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiStatusJson()); });
  wifiServer.on("/api/networks", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiNetworksJson()); });
  wifiServer.on("/api/scan", HTTP_GET, []() { wifiServer.send(200, "application/json", wifiScanJson()); });
  wifiServer.on("/api/measurement/status", HTTP_GET, []() { wifiServer.send(200, "application/json", measurementStatusJson()); });
  wifiServer.on("/api/measurement/mode", HTTP_POST, []() { handleMeasurementModeApi(); });
  wifiServer.on("/api/measurement/cv", HTTP_POST, []() { handleCvConfigApi(); });
  wifiServer.on("/api/measurement/swv", HTTP_POST, []() { handleSwvConfigApi(); });
  wifiServer.on("/api/measurement/dpv", HTTP_POST, []() { handleDpvConfigApi(); });
  wifiServer.on("/api/measurement/start", HTTP_POST, []() { handleMeasurementStartApi(); });
  wifiServer.on("/api/measurement/stop", HTTP_POST, []() { handleMeasurementStopApi(); });
  wifiServer.on("/api/measurement/precheck", HTTP_POST, []() { handleContactPrecheckApi(); });
  wifiServer.on("/api/measurement/vibration-test", HTTP_POST, []() { handleVibrationTestApi(); });
  wifiServer.on("/api/cv/files", HTTP_GET, []() { handleCvFilesApi(); });
  wifiServer.on("/api/cv/download", HTTP_GET, []() { handleCvDownloadCsvApi(); });
  wifiServer.on("/api/cv/download-bin", HTTP_GET, []() { handleCvDownloadBinApi(); });
  wifiServer.on("/api/cv/delete", HTTP_POST, []() { handleCvDeleteApi(); });
  wifiServer.on("/api/assets/status", HTTP_GET, []() { wifiServer.send(200, "application/json", sdAssetStatusJson()); });
  wifiServer.on("/api/assets/upload-plotly", HTTP_POST, []() { handlePlotlyUploadResult(); }, []() { handlePlotlyUploadStream(); });
  wifiServer.on("/api/ota/version", HTTP_GET, []() { handleOtaVersionApi(); });
  wifiServer.on("/api/ota/firmware", HTTP_POST, []() { handleOtaFirmwareApi(); });
  wifiServer.on("/api/ota/web", HTTP_POST, []() { handleOtaWebApi(); });
  wifiServer.on("/api/ota/all", HTTP_POST, []() { handleOtaAllApi(); });
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
  Serial.println("WiFi web server started.");
}

static void updateWifiMdns() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiMdnsStarted) return;

    String host = wifiMdnsHost();
    if (MDNS.begin(host.c_str())) {
      MDNS.addService("http", "tcp", 80);
      wifiMdnsStarted = true;
      Serial.print("mDNS started: ");
      Serial.println(wifiMdnsUrl());
    } else {
      Serial.println("mDNS start failed.");
    }
    return;
  }

  if (wifiMdnsStarted) {
    MDNS.end();
    wifiMdnsStarted = false;
    Serial.println("mDNS stopped.");
  }
}

static void startWifiPortal() {
  wifiApSsid = String("smart-ec-Setup-") + wifiDeviceIdSuffix();

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

  startWifiWebServer();

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

static void requestTftRefreshOnStatusChange() {
#if ENABLE_TFT_DISPLAY
  static int lastWifiStatus = -1;
  static String lastWifiSsid;
  static String lastWifiIp;
  static bool lastApMode = false;
  static String lastApSsid;
  static bool lastSdMounted = false;
  static bool lastRtcAvailable = false;
  static bool lastRtcUtcSynced = false;

  int wifiStatus = WiFi.status();
  String wifiSsid = (wifiStatus == WL_CONNECTED) ? WiFi.SSID() : String("");
  String wifiIp = (wifiStatus == WL_CONNECTED) ? WiFi.localIP().toString() : String("");

  if (wifiStatus != lastWifiStatus ||
      wifiSsid != lastWifiSsid ||
      wifiIp != lastWifiIp ||
      wifiApMode != lastApMode ||
      wifiApSsid != lastApSsid ||
      sdMounted != lastSdMounted ||
      rtcAvailable != lastRtcAvailable ||
      rtcUtcSynced != lastRtcUtcSynced) {
    lastWifiStatus = wifiStatus;
    lastWifiSsid = wifiSsid;
    lastWifiIp = wifiIp;
    lastApMode = wifiApMode;
    lastApSsid = wifiApSsid;
    lastSdMounted = sdMounted;
    lastRtcAvailable = rtcAvailable;
    lastRtcUtcSynced = rtcUtcSynced;
    requestTftStatusRefresh();
  }
#endif
}

static void requestTftPeriodicIdleRefresh() {
#if ENABLE_TFT_DISPLAY
  static uint32_t lastRefreshMs = 0;
  if (measurementSessionActive()) return;

  uint32_t nowMs = millis();
  if (nowMs - lastRefreshMs < 60000UL) return;
  lastRefreshMs = nowMs;
  requestTftTimeRefresh();
#endif
}

static void wifiManagerBeginFromOptionTask() {
  Serial.println("WiFi manager: default first, then fast-scan NVS credentials, then AP portal.");
  loadInstalledWebVersion();
  if (connectDefaultWifiFirst()) {
    startWifiWebServer();
    return;
  }
  if (connectStoredWifiFast()) {
    startWifiWebServer();
    return;
  }
  startWifiPortal();
}

static void wifiManagerLoop() {
  if (wifiServerStarted) {
    wifiDns.processNextRequest();
    wifiServer.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED) {
    startWifiWebServer();
    stopWifiPortalIfConnected();
    updateWifiMdns();
    maybeSyncUtcAfterWifiConnected();
    requestTftRefreshOnStatusChange();
    return;
  }

  updateWifiMdns();

  uint32_t nowMs = millis();
  if (wifiConnectRequested || (!wifiApMode && nowMs - wifiLastRetryMs > WIFI_RETRY_PERIOD_MS)) {
    wifiConnectRequested = false;
    wifiLastRetryMs = nowMs;
    if (!connectDefaultWifiFirst() && !connectStoredWifiFast()) {
      startWifiPortal();
    }
  }
  requestTftRefreshOnStatusChange();
}

static bool initSdCard() {
#if ENABLE_SD_LOGGING
  if (sdMounted) return true;

  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0, SD_D1, SD_D2, SD_D3);
  if (!SD_MMC.begin("/sdcard", false)) {
    sdMounted = false;
    Serial.println("SD_MMC 4-bit mount failed. CV .bin logging disabled.");
    requestTftFullRefresh();
    return false;
  }

  sdMounted = true;
  Serial.print("SD_MMC 4-bit mounted OK, size MB=");
  Serial.println((uint32_t)(SD_MMC.cardSize() / (1024ULL * 1024ULL)));
  requestTftFullRefresh();
  return true;
#else
  return false;
#endif
}

void sdWriterTask(void *pvParameters) {
  (void)pvParameters;

#if ENABLE_SD_LOGGING
  File file;
  bool headerWritten = false;
  uint32_t lastFlushMs = millis();
  CvBinRecord batch[SD_WRITE_BATCH_LIMIT];

  Serial.println("SDWriterTask started on Core " + String(xPortGetCoreID()));

  for (;;) {
    if (!sdMounted) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    if (!cvLogActive && !file) {
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (cvLogActive && !file) {
      file = SD_MMC.open(activeCvFilePath, FILE_WRITE);
      headerWritten = false;
      if (!file) {
        sdWriteErrorCount++;
        Serial.print("CV log open failed: ");
        Serial.println(activeCvFilePath);
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }

      size_t written = file.write((const uint8_t *)&activeCvHeader, sizeof(activeCvHeader));
      headerWritten = (written == sizeof(activeCvHeader));
      sdLogFileOpen = headerWritten;
      if (!headerWritten) {
        sdWriteErrorCount++;
        file.close();
        sdLogFileOpen = false;
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }

      vTaskDelay(1);
    }

    if (file && headerWritten) {
      uint8_t recordsWrittenThisPass = 0;
      TickType_t waitTicks = pdMS_TO_TICKS(20);

      while (recordsWrittenThisPass < SD_WRITE_BATCH_LIMIT &&
             xQueueReceive(sdRecordQueue, &batch[recordsWrittenThisPass], waitTicks) == pdTRUE) {
        recordsWrittenThisPass++;
        waitTicks = 0;
      }

      if (recordsWrittenThisPass > 0) {
        size_t bytesToWrite = recordsWrittenThisPass * sizeof(CvBinRecord);
        size_t written = file.write((const uint8_t *)batch, bytesToWrite);
        if (written == bytesToWrite) {
          sdRecordWriteCount += recordsWrittenThisPass;
        } else {
          sdWriteErrorCount++;
        }
      }
    }

    uint32_t nowMs = millis();
    if (file && nowMs - lastFlushMs >= SD_FLUSH_INTERVAL_MS) {
      file.flush();
      lastFlushMs = nowMs;
    }

    if (file && !cvLogActive && uxQueueMessagesWaiting(sdRecordQueue) == 0) {
      file.flush();
      file.close();
      sdLogFileOpen = false;
      Serial.print("CV log closed: ");
      Serial.print(activeCvFilePath);
      Serial.print(" records=");
      Serial.println(sdRecordWriteCount);
    }

    vTaskDelay(1);
  }
#else
  vTaskDelete(nullptr);
#endif
}



void optionTask(void *pvParameters) {
  Serial.println("OptionTask started on Core " + String(xPortGetCoreID()) + " (low priority, Serial/logger/options/WiFi manager)");
  updateSerialDebugMode();

  MeasurementData data;
  uint32_t lastI2CStatusMs = 0;

  wifiManagerBeginFromOptionTask();
  requestTftFullRefresh();

  while (true) {
    updateSerialDebugMode();

    // 1) Serial commands
    while (Serial.available()) {
      updateSerialDebugMode(true);
      handleSerialCommand((char)Serial.read());
    }

#if ENABLE_TFT_DISPLAY && ENABLE_TOUCH_CONTROL && ENABLE_TOUCH_POLLING
    handleTouchControl();
#endif

    // 2) Logger: drain a limited number per loop to avoid CPU0 WDT / Serial blocking too long.
    if (serialDebugMode) {
      if (!serialCsvHeaderPrinted) {
        Serial.println("mode,direction,DAC_set_V,DAC_code,DAC_expected_V,ADC_raw,ADC_V,ADC_delta_V,current_A,RTIA_ohm,TIACN,AutoRange,timestamp_us,period_us");
        serialCsvHeaderPrinted = true;
      }

      uint8_t printed = 0;
      while (printed < 4 && xQueueReceive(loggerQueue, &data, 0) == pdTRUE) {
        printMeasurementCsv(data);
        printed++;
      }
    } else if (loggerQueue) {
      xQueueReset(loggerQueue);
      serialCsvHeaderPrinted = false;
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
    requestTftPeriodicIdleRefresh();
    serviceTftRefresh();
#endif

    wifiManagerLoop();

    // Always yield. OptionTask must never starve the idle task.
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
// ======================================================
// Vibration motor control (simple on/off for now, no PWM).
// ======================================================
void vibrationBegin() {
  pinMode(VIB_MOTOR_PIN, OUTPUT);

  // Arduino-ESP32 core ใหม่
  // ledcAttach(pin, freq, resolution)
  ledcAttach(VIB_MOTOR_PIN, VIB_PWM_FREQ, VIB_PWM_RES);
  ledcWrite(VIB_MOTOR_PIN, 0);   // OFF
}

void vibrationOn(uint8_t duty) {
  // duty 0-255
  ledcWrite(VIB_MOTOR_PIN, duty);
}

void vibrationOff() {
  ledcWrite(VIB_MOTOR_PIN, 0);
}
void vibrationPulse(uint8_t duty, uint32_t durationMs) {
  // Start kick
  ledcWrite(VIB_MOTOR_PIN, 255);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Run
  ledcWrite(VIB_MOTOR_PIN, duty);
  vTaskDelay(pdMS_TO_TICKS(durationMs));

  // Stop
  ledcWrite(VIB_MOTOR_PIN, 0);
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
  esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.print("Reset reason: ");
  Serial.println(resetReasonToText(resetReason));
  Serial.println();

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);

  vibrationBegin();
  vibrationPulse(30, 300); 
  
  // ------------------------------
  // FreeRTOS objects first
  // ------------------------------
  spiMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  loggerQueue = xQueueCreate(256, sizeof(MeasurementData));
  sdRecordQueue = xQueueCreate(1024, sizeof(CvBinRecord));

  if (!spiMutex || !i2cMutex || !loggerQueue || !sdRecordQueue) {
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
  rtcProbe_safe();
  Serial.print("RTC PCF85363A: ");
  Serial.println(rtcAvailable ? "found" : "not found");

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

#if ENABLE_SD_LOGGING
  initSdCard();
#endif

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
  Serial.println("  r = run measurement, reset RTIA start range first");
  Serial.println("  s = stop measurement");
  Serial.println("  1 = select SWEEP mode");
  Serial.println("  2 = select HOLD_CENTER mode (2.048V repeated)");
  Serial.println("  3 = select CV mode (web parameters)");
  Serial.println("  4 = select SWV mode (web parameters)");
  Serial.println("  5 = select DPV mode (web parameters)");
  Serial.println("  m = print selected measurement mode");
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
    sdWriterTask,
    "SDWriterTask",
    8192,
    nullptr,
    OPTION_PRIORITY,
    &sdWriterTaskHandle,
    0
  );

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
