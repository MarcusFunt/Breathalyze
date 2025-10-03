/*
  XIAO ESP32S3 + BMP280 (breath trigger) + MQ303A (alcohol) -> Gadgetbridge over BLE
  Wiring (XIAO pin -> function):
    D4 / GPIO5  -> I2C SDA (BMP280)
    D5 / GPIO6  -> I2C SCL (BMP280)
    D3 / GPIO4  -> MQ303A DAT (analog to MCU, 0..3.3 V)
    D6 / GPIO43 -> MQ303A heater enable (LOW = HEAT ON per your schematic)
    3V3         -> MQ303A board VCC
    GND         -> MQ303A GND and BMP280 GND
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <NimBLEDevice.h>
#include <stdio.h>
#include <math.h>
#include <algorithm>

// ===== Pins: Seeed XIAO ESP32S3 =====
#define I2C_SDA        5    // D4
#define I2C_SCL        6    // D5
#define ALC_DAT_PIN    4    // D3 / A3  (0..3.3 V from your board)
#define ALC_SEL_PIN    43   // D6       (LOW = heater ON)

// ===== Breath detection + sampling tuning =====
static const float    SAMPLE_HZ          = 50.0f;   // BMP280 poll rate
static const float    BASELINE_TAU_S     = 15.0f;   // EMA time-constant for ambient drift
static const float    TRIG_DELTA_HPA     = 1.0f;    // threshold to declare "blow" (≈100 Pa)
static const uint16_t TRIG_SUSTAIN_MS    = 150;     // must exceed threshold for this long
static const uint32_t SAMPLE_WINDOW_MS   = 5000;    // read alcohol for this long
static const uint32_t REFRACTORY_MS      = 4000;    // ignore new blows after a sample

// Heater policy: always on gives best repeatability
static const bool     HEATER_ALWAYS_ON   = true;    // set false to warm only after trigger
static const uint32_t QUICK_HEAT_MS      = 10000;   // warmup time if not always on

// ===== Sensors =====
Adafruit_BMP280 bmp;
bool bmpOK = false;

// ===== Gadgetbridge BLE (Nordic UART Service compatible) =====
static NimBLECharacteristic* gbTxCharacteristic = nullptr;
static bool gbConnected = false;
static bool gbNotifyEnabled = false;
static uint16_t gbNotifyMax = 20;

// ===== State machine =====
enum Mode { IDLE, WARMING, SAMPLING, REFRACTORY } mode = IDLE;

float baselinePa = NAN;
float lpAlpha = 0.0f;           // EMA coefficient
uint32_t lastSampleMS = 0;

uint16_t trigHoldMS = 0;
uint32_t modeTS = 0;

int   alcRaw  = 0, alcPeak = 0;
float lastDeltaHPA = 0.0f;
bool lastPressureValid = false;

void gbSendStatus(const String &msg);
void gbSendLog(const String &msg);
void gbSendResult(int peakRaw, float peakVoltage);
void gbSendInitialBurst();
String gbCurrentStatusMessage();

// BLE UUIDs used by Gadgetbridge/Bangle.js Nordic UART service
static const NimBLEUUID GB_SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static const NimBLEUUID GB_CHAR_UUID_RX("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static const NimBLEUUID GB_CHAR_UUID_TX("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

class GBServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    gbConnected = true;
    gbNotifyEnabled = false;
    gbNotifyMax = 20;
    gbSendStatus(gbCurrentStatusMessage());
  }
  void onDisconnect(NimBLEServer* server) override {
    gbConnected = false;
    gbNotifyEnabled = false;
    gbNotifyMax = 20;
    NimBLEDevice::startAdvertising();
  }
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override {
    int adjusted = static_cast<int>(MTU) - 3;
    if (adjusted < 20) adjusted = 20;
    gbNotifyMax = static_cast<uint16_t>(adjusted);
  }
};

class GBRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    // Gadgetbridge may send "v" (version) or other commands. We do not
    // currently act on them, but reading and ignoring prevents warnings.
    std::string value = characteristic->getValue();
    (void)value;
  }
};

class GBTxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* characteristic,
                   ble_gap_conn_desc* desc,
                   uint16_t subValue) override {
    bool enabled = (subValue & 0x0001);
    gbNotifyEnabled = enabled;
    if (enabled) {
      gbSendInitialBurst();
    }
  }
};

String gbEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    switch (c) {
      case '\\': out += "\\\\"; break;
      case '\"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<uint8_t>(c) < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", c & 0xFF);
          out += buf;
        } else {
          out += c;
        }
        break;
    }
  }
  return out;
}

void gbSendJSON(const String &json) {
  if (!gbConnected || gbTxCharacteristic == nullptr || !gbNotifyEnabled) return;
  String payload = "GB(" + json + ")\n";
  const char* data = payload.c_str();
  size_t totalLen = payload.length();
  for (size_t offset = 0; offset < totalLen; offset += gbNotifyMax) {
    size_t chunkLen = std::min<size_t>(gbNotifyMax, totalLen - offset);
    if (!gbTxCharacteristic->setValue(reinterpret_cast<const uint8_t*>(data + offset),
                                      chunkLen)) {
      Serial.println("GB notify setValue failed");
      return;
    }
    if (!gbTxCharacteristic->notify()) {
      Serial.println("GB notify failed");
      return;
    }
    delay(5);  // allow stack to flush before next packet
  }
}

String gbCurrentStatusMessage() {
  switch (mode) {
    case IDLE:
      return String("READY");
    case SAMPLING:
      return String("SAMPLING");
    case REFRACTORY:
      return String("RESULT");
    case WARMING: {
      uint32_t now = millis();
      uint32_t elapsed = now - modeTS;
      uint32_t remainS = (elapsed >= QUICK_HEAT_MS)
                             ? 0
                             : (QUICK_HEAT_MS - elapsed + 999) / 1000;
      return String("WARMING ") + remainS + "s";
    }
  }
  return String("READY");
}

void gbSendStatus(const String &msg) {
  String json = String("{\"t\":\"status\",\"src\":\"Breathalyze\",\"msg\":\"") +
                gbEscape(msg) + "\"}";
  gbSendJSON(json);
}

void gbSendLog(const String &msg) {
  String json = String("{\"t\":\"log\",\"src\":\"Breathalyze\",\"msg\":\"") +
                gbEscape(msg) + "\"}";
  gbSendJSON(json);
}

void gbSendResult(int peakRaw, float peakVoltage) {
  String body = String("Peak: ") + peakRaw + " (" + String(peakVoltage, 2) + " V)";
  String json = String("{\"t\":\"notify\",\"id\":1,\"src\":\"Breathalyze\",\"title\":\"Breathalyzer\",\"body\":\"") +
                gbEscape(body) + "\"}";
  gbSendJSON(json);
}

void gbSendInitialBurst() {
  gbSendStatus(gbCurrentStatusMessage());
  if (bmpOK && !isnan(baselinePa)) {
    gbSendLog(String("PRESS ") + String(lastDeltaHPA, 2) + " hPa");
  } else {
    gbSendLog("PRESS --");
  }
  float v = (alcRaw / 4095.0f) * 3.3f;
  gbSendLog(String("ALC ") + alcRaw + " (" + String(v, 2) + " V)");
  gbSendLog(String("PEAK ") + alcPeak);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // --- I2C / BMP280 ---
  Wire.begin(I2C_SDA, I2C_SCL);
  bmpOK = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpOK) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,   // temp
                    Adafruit_BMP280::SAMPLING_X16,  // pressure (good SNR)
                    Adafruit_BMP280::FILTER_X16,    // smooth turbulence
                    Adafruit_BMP280::STANDBY_MS_1); // <-- BMP280-valid standby
  } else {
    Serial.println("BMP280 not found at 0x76/0x77. Breath trigger disabled.");
  }

  // --- MQ303A heater control (your board: LOW = HEAT ON) ---
  pinMode(ALC_SEL_PIN, OUTPUT);
  digitalWrite(ALC_SEL_PIN, HEATER_ALWAYS_ON ? LOW : HIGH);

  // --- ADC config for 0..3.3 V full-scale on ESP32-S3 ---
  analogReadResolution(12);                         // 0..4095
  analogSetPinAttenuation(ALC_DAT_PIN, ADC_11db);   // ~3.3 V FS

  // --- Gadgetbridge BLE ---
  NimBLEDevice::init("Breathalyze");
  NimBLEDevice::setPower(ESP_PWR_LVL_P7);
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new GBServerCallbacks());

  NimBLEService* service = server->createService(GB_SERVICE_UUID);
  gbTxCharacteristic = service->createCharacteristic(
      GB_CHAR_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY
  );
  gbTxCharacteristic->setCallbacks(new GBTxCallbacks());
  NimBLECharacteristic* rxCharacteristic = service->createCharacteristic(
      GB_CHAR_UUID_RX,
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxCharacteristic->setCallbacks(new GBRxCallbacks());

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(GB_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinInterval(100);
  advertising->setMaxInterval(200);
  NimBLEDevice::startAdvertising();

  // Prime UI
  gbSendStatus("READY");
  gbSendLog("PRESS --");
  gbSendLog("ALC --");
  gbSendLog("PEAK --");

  // EMA coefficient for baseline tracking
  float dt = 1.0f / SAMPLE_HZ;
  lpAlpha  = dt / BASELINE_TAU_S;

  lastSampleMS = millis();
}

void beginSamplingWindow() {
  mode = SAMPLING;
  modeTS = millis();
  alcPeak = 0;
  gbSendStatus("SAMPLING");
}

void loop() {
  // no dedicated BLE service loop required for NimBLE-Arduino

  // --- fixed-rate sampling (BMP280 + logic) ---
  uint32_t now = millis();
  if ((now - lastSampleMS) < (uint32_t)(1000.0f / SAMPLE_HZ)) return;
  lastSampleMS = now;

  // ----- Read pressure & update baseline -----
  static bool announcedNoPress = false;
  if (bmpOK) {
    static uint32_t lastPressureReport = 0;
    static uint32_t lastInvalidPressureLog = 0;

    float pPa = bmp.readPressure();                 // Pascals
    bool pressureValid = !isnan(pPa);
    if (!pressureValid) {
      if (!announcedNoPress || (now - lastInvalidPressureLog >= 1000)) {
        gbSendLog("PRESS --");
        lastInvalidPressureLog = now;
        announcedNoPress = true;
      }
      lastDeltaHPA = 0.0f;
      trigHoldMS = 0;
    } else {
      if (isnan(baselinePa)) baselinePa = pPa;        // initialize

      baselinePa = baselinePa + lpAlpha * (pPa - baselinePa);  // EMA
      float deltaHPA = (pPa - baselinePa) / 100.0f;            // hPa
      lastDeltaHPA = deltaHPA;

      // Show live pressure delta (helps tune threshold)
      if (now - lastPressureReport >= 500) { // limit to 2 Hz
        gbSendLog(String("PRESS ") + String(deltaHPA, 2) + " hPa");
        lastPressureReport = now;
      }
      announcedNoPress = false;
    }
    lastPressureValid = pressureValid;
  } else {
    if (!announcedNoPress) {
      gbSendLog("PRESS --");
      announcedNoPress = true;
    }
    lastDeltaHPA = 0.0f;
    trigHoldMS = 0;
    lastPressureValid = false;
  }

  // ----- State machine -----
  switch (mode) {
    case IDLE: {
      if (!HEATER_ALWAYS_ON) digitalWrite(ALC_SEL_PIN, HIGH); // heater OFF
      if (bmpOK) {
        if (!lastPressureValid) {
          trigHoldMS = 0;
          break;
        }
        // Require breath threshold for TRIG_SUSTAIN_MS
        if (lastDeltaHPA > TRIG_DELTA_HPA) {
          trigHoldMS += (uint16_t)(1000.0f / SAMPLE_HZ);
          if (trigHoldMS >= TRIG_SUSTAIN_MS) {
            if (!HEATER_ALWAYS_ON) {
              digitalWrite(ALC_SEL_PIN, LOW); // heater ON
              mode = WARMING;
              modeTS = now;
              gbSendStatus("WARMING...");
            } else {
              beginSamplingWindow();
            }
            trigHoldMS = 0;
          }
        } else {
          trigHoldMS = 0;
        }
      }
      break;
    }

    case WARMING: {
      uint32_t elapsed = now - modeTS;
      uint32_t remainS = (elapsed >= QUICK_HEAT_MS) ? 0 : (QUICK_HEAT_MS - elapsed + 999) / 1000;
      static uint32_t lastAnnounce = 0;
      if (now - lastAnnounce > 500) {
        gbSendStatus(String("WARMING ") + remainS + "s");
        lastAnnounce = now;
      }
      if (elapsed >= QUICK_HEAT_MS) beginSamplingWindow();
      break;
    }

    case SAMPLING: {
      // Read alcohol (0..4095); report live value and track peak
      int raw = analogRead(ALC_DAT_PIN);
      alcRaw = raw;
      if (raw > alcPeak) alcPeak = raw;

      float v = (raw / 4095.0f) * 3.3f;
      static uint32_t lastAlcReport = 0;
      if (now - lastAlcReport >= 250) { // 4 Hz updates while sampling
        gbSendLog(String("ALC ") + raw + " (" + String(v, 2) + " V)");
        lastAlcReport = now;
      }

      if (now - modeTS >= SAMPLE_WINDOW_MS) {
        float peakVoltage = (alcPeak / 4095.0f) * 3.3f;
        gbSendLog(String("PEAK ") + alcPeak);
        gbSendStatus("RESULT");
        gbSendResult(alcPeak, peakVoltage);
        mode = REFRACTORY;
        modeTS = now;
        if (!HEATER_ALWAYS_ON) digitalWrite(ALC_SEL_PIN, HIGH); // heater OFF
      }
      break;
    }

    case REFRACTORY: {
      if (now - modeTS >= REFRACTORY_MS) {
        gbSendStatus("READY");
        mode = IDLE;
      }
      break;
    }
  }
}
