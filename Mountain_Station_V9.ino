#include <Wire.h>
#include <Adafruit_BME280.h>
#include <RadioLib.h>
#include <SPI.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include "esp_sleep.h"

// SENSOR OBJECTS
NAU7802 nau;
Adafruit_BME280 bme;
TwoWire I2CBUS = TwoWire(0);

// I2C CONFIG
static const int I2C_SDA = 41;
static const int I2C_SCL = 42;

// HELTEC V3 LORA PINS
static const int LORA_NSS  = 8;
static const int LORA_DIO1 = 14;
static const int LORA_RST  = 12;
static const int LORA_BUSY = 13;

static const int LORA_SCK  = 9;
static const int LORA_MISO = 11;
static const int LORA_MOSI = 10;

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);

// LORA SETTINGS
static const float LORA_FREQ_MHZ = 868.0;
static const int   LORA_SF = 7;
static const float LORA_BW_KHZ = 125.0;
static const int   LORA_CR = 5;
static const int   LORA_TX_POWER = 14;

// TX INTERVALS
static const unsigned long TX_INTERVAL_NORMAL_MS = 30UL * 60UL * 1000UL;  // 30 min
static const unsigned long TX_INTERVAL_ALARM_MS  = 5UL  * 60UL * 1000UL;  // 5 min

// ALARM THRESHOLDS
static const float TEMP_ALARM_C    = 0.0;    // Alarm if temperature >= 0°C
static const float HUM_ALARM_RH    = 85.0;   // Alarm if humidity >= 85%
static const float DISP_ALARM_MM   = 1.00;   // Alarm if displacement >= 1mm 
// Hysteresis
static const float TEMP_HYST_C     = 0.5;
static const float HUM_HYST_RH     = 2.0;
static const float DISP_HYST_MM    = 0.10;

// DMS / NAU7802 SETTINGS
static const int DMS_SAMPLES = 16;

// displacement_mm = (counts - offsetCounts) * DMS_mm_per_count
static long  DMS_offsetCounts = 320996;      
static float DMS_mm_per_count = 0.0000085f;   // conversion factor from raw counts to displacement in mm

// RTC-PERSISTENT STATE
RTC_DATA_ATTR bool alarmMode = false;
RTC_DATA_ATTR uint32_t bootCount = 0;

// HELPER FUNCTIONS
static bool readDMSCountsAvg(long &avgCounts) {
  long sum = 0;
  int got = 0;

  unsigned long t0 = millis();
  while (got < DMS_SAMPLES && (millis() - t0) < 200) {
    if (nau.available()) {
      sum += nau.getReading();
      got++;
    }
  }

  if (got == 0) return false;
  avgCounts = sum / got;
  return true;
}

static float countsToDisplacementMm(long counts) {
  return (counts - DMS_offsetCounts) * DMS_mm_per_count;
}

static void printI2CScan() {
  Serial.println("I2C scan...");
  int found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    I2CBUS.beginTransmission(addr);
    if (I2CBUS.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }

  Serial.print("I2C devices found: ");
  Serial.println(found);
}

static bool initBME280() {
  if (bme.begin(0x76, &I2CBUS)) {
    Serial.println("BME280 found at 0x76");
    return true;
  }

  if (bme.begin(0x77, &I2CBUS)) {
    Serial.println("BME280 found at 0x77");
    return true;
  }

  return false;
}

static bool initNAU7802() {
  if (!nau.begin(I2CBUS)) {
    return false;
  }

  nau.setGain(NAU7802_GAIN_128);
  nau.setSampleRate(NAU7802_SPS_40);

  if (!nau.calibrateAFE()) {
    Serial.println("NAU7802 AFE calibration failed");
    return false;
  }

  return true;
}

static bool shouldEnterAlarm(float t, float h, float displacement_mm, bool dmsOk) {
  bool enterAlarm = false;

  if (t >= TEMP_ALARM_C) enterAlarm = true;
  if (h >= HUM_ALARM_RH) enterAlarm = true;
  if (dmsOk && displacement_mm >= DISP_ALARM_MM) enterAlarm = true;

  return enterAlarm;
}

static bool shouldExitAlarm(float t, float h, float displacement_mm, bool dmsOk) {
  bool exitAlarm = true;

  exitAlarm = exitAlarm && (t < (TEMP_ALARM_C - TEMP_HYST_C));
  exitAlarm = exitAlarm && (h < (HUM_ALARM_RH - HUM_HYST_RH));

  if (dmsOk) {
    exitAlarm = exitAlarm && (displacement_mm < (DISP_ALARM_MM - DISP_HYST_MM));
  }

  return exitAlarm;
}

static String buildPayload(float t, float h, float p, bool dmsOk, long dmsCounts,
                           float displacement_mm, bool alarm) {
  String payload = String("T=") + String(t, 2) +
                   ",H=" + String(h, 2) +
                   ",P=" + String(p, 2);

  if (dmsOk) {
    payload += ",DMS=" + String(dmsCounts);
    payload += ",d=" + String(displacement_mm, 2);
  } else {
    payload += ",DMS=NaN,d=NaN";
  }

  payload += alarm ? ",AL=1" : ",AL=0";

  return payload;
}

static bool initLoRa() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  int state = radio.begin(LORA_FREQ_MHZ);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed, code = ");
    Serial.println(state);
    return false;
  }

  radio.setOutputPower(LORA_TX_POWER);
  radio.setSpreadingFactor(LORA_SF);
  radio.setBandwidth(LORA_BW_KHZ);
  radio.setCodingRate(LORA_CR);

  Serial.println("LoRa TX ready");
  return true;
}

static void enterDeepSleepMs(unsigned long sleepMs) {
  Serial.print("Entering deep sleep for ");
  Serial.print(sleepMs);
  Serial.println(" ms");

  Serial.flush();

  esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  esp_deep_sleep_start();
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(500);

  bootCount++;
  Serial.println("Starting Mountain Station...");
  Serial.print("Boot count: ");
  Serial.println(bootCount);

  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup reason: ");
  Serial.println((int)wakeupReason);

  // I2C
  I2CBUS.begin(I2C_SDA, I2C_SCL, 100000);
  printI2CScan();

  // BME280
  if (!initBME280()) {
    Serial.println("BME280 not found. Check wiring.");
    enterDeepSleepMs(TX_INTERVAL_NORMAL_MS);
  }

  // NAU7802
  if (!initNAU7802()) {
    Serial.println("NAU7802 not detected. Check wiring.");
    enterDeepSleepMs(TX_INTERVAL_NORMAL_MS);
  }

  Serial.println("NAU7802 OK");

  // LoRa
  if (!initLoRa()) {
    enterDeepSleepMs(TX_INTERVAL_NORMAL_MS);
  }

  Serial.println("Mountain Station ready");

  // Read BME280
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0F;

  // Read DMS via NAU7802
  long dmsCounts = 0;
  bool dmsOk = readDMSCountsAvg(dmsCounts);
  float displacement_mm = dmsOk ? countsToDisplacementMm(dmsCounts) : NAN;

  Serial.print("DMS counts: ");
  Serial.println(dmsCounts);

  Serial.print("Displacement (mm): ");
  if (dmsOk) {
    Serial.println(displacement_mm, 2);
  } else {
    Serial.println("NaN");
  }

  // Alarm logic
  bool enterAlarm = shouldEnterAlarm(t, h, displacement_mm, dmsOk);
  bool exitAlarm  = shouldExitAlarm(t, h, displacement_mm, dmsOk);

  if (!alarmMode && enterAlarm) {
    alarmMode = true;
    Serial.println(">>> ALARM MODE ON");
  }
  else if (alarmMode && exitAlarm) {
    alarmMode = false;
    Serial.println(">>> ALARM MODE OFF");
  }

  // Build and send payload
  String payload = buildPayload(t, h, p, dmsOk, dmsCounts, displacement_mm, alarmMode);

  Serial.print("Sending: ");
  Serial.println(payload);

  int state = radio.transmit(payload);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("TX OK");
  } else {
    Serial.print("TX failed, code = ");
    Serial.println(state);
  }

  // Select sleep interval by mode
  unsigned long nextSleepMs = alarmMode ? TX_INTERVAL_ALARM_MS : TX_INTERVAL_NORMAL_MS;

  if (alarmMode) {
    Serial.println("Next wake-up in ALARM interval");
  } else {
    Serial.println("Next wake-up in NORMAL interval");
  }

  enterDeepSleepMs(nextSleepMs);
}

void loop() {
}