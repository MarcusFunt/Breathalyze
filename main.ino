/*
  XIAO ESP32S3 + BMP280 (breath trigger) + MQ303A (alcohol) -> DashIO over BLE
  Wiring (XIAO pin -> function):
    D4 / GPIO5  -> I2C SDA (BMP280)
    D5 / GPIO6  -> I2C SCL (BMP280)
    D3 / GPIO4  -> MQ303A DAT (analog to MCU, 0..3.3 V)
    D6 / GPIO43 -> MQ303A heater enable (LOW = HEAT ON per your schematic)
    3V3         -> MQ303A board VCC
    GND         -> MQ303A GND and BMP280 GND

  DashIO UI: add 4 Text Boxes with IDs STAT, PRESS, ALC, PEAK (or import your own layout)
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "DashioESP.h"   // DashIoT/DashioESP

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

// ===== DashIO BLE =====
DashDevice dashDevice("XIAO_ESP32S3");
DashBLE    ble_con(&dashDevice, true);

// Helpers to send text box updates (batch into one BLE message)
void sendText(const char* id, const String &txt) {
  static String buf;
  buf += dashDevice.getTextBoxMessage(id, txt);
  if (id == nullptr || buf.length() > 400) {
    ble_con.sendMessage(buf);
    buf = "";
  }
}

// ===== State machine =====
enum Mode { IDLE, WARMING, SAMPLING, REFRACTORY } mode = IDLE;

float baselinePa = NAN;
float lpAlpha = 0.0f;           // EMA coefficient
uint32_t lastSampleMS = 0;

uint16_t trigHoldMS = 0;
uint32_t modeTS = 0;

int   alcRaw  = 0, alcPeak = 0;
float lastDeltaHPA = 0.0f;

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

  // --- DashIO BLE ---
  ble_con.begin();  // starts advertising

  // Prime UI
  sendText("STAT",  "READY");
  sendText("PRESS", "--");
  sendText("ALC",   "--");
  sendText("PEAK",  "--");
  sendText(nullptr, ""); // flush

  // EMA coefficient for baseline tracking
  float dt = 1.0f / SAMPLE_HZ;
  lpAlpha  = dt / BASELINE_TAU_S;

  lastSampleMS = millis();
}

void beginSamplingWindow() {
  mode = SAMPLING;
  modeTS = millis();
  alcPeak = 0;
  sendText("STAT", "SAMPLING");
  sendText(nullptr, "");
}

void loop() {
  ble_con.run(); // keep BLE happy

  // --- fixed-rate sampling (BMP280 + logic) ---
  uint32_t now = millis();
  if ((now - lastSampleMS) < (uint32_t)(1000.0f / SAMPLE_HZ)) return;
  lastSampleMS = now;

  // ----- Read pressure & update baseline -----
  if (bmpOK) {
    float pPa = bmp.readPressure();                 // Pascals
    if (isnan(baselinePa)) baselinePa = pPa;        // initialize

    baselinePa = baselinePa + lpAlpha * (pPa - baselinePa);  // EMA
    float deltaHPA = (pPa - baselinePa) / 100.0f;            // hPa
    lastDeltaHPA = deltaHPA;

    // Show live pressure delta (helps tune threshold)
    sendText("PRESS", String(deltaHPA, 2) + " hPa");
  } else {
    sendText("PRESS", "--");
  }

  // ----- State machine -----
  switch (mode) {
    case IDLE: {
      if (!HEATER_ALWAYS_ON) digitalWrite(ALC_SEL_PIN, HIGH); // heater OFF
      if (bmpOK) {
        // Require breath threshold for TRIG_SUSTAIN_MS
        if (lastDeltaHPA > TRIG_DELTA_HPA) {
          trigHoldMS += (uint16_t)(1000.0f / SAMPLE_HZ);
          if (trigHoldMS >= TRIG_SUSTAIN_MS) {
            if (!HEATER_ALWAYS_ON) {
              digitalWrite(ALC_SEL_PIN, LOW); // heater ON
              mode = WARMING;
              modeTS = now;
              sendText("STAT", "WARMING...");
              sendText(nullptr, "");
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
        sendText("STAT", String("WARMING ") + remainS + "s");
        sendText(nullptr, "");
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
      sendText("ALC", String(raw) + " (" + String(v, 2) + " V)");
      sendText(nullptr, "");

      if (now - modeTS >= SAMPLE_WINDOW_MS) {
        sendText("PEAK", String(alcPeak));
        sendText("STAT", "RESULT");
        sendText(nullptr, "");
        mode = REFRACTORY;
        modeTS = now;
        if (!HEATER_ALWAYS_ON) digitalWrite(ALC_SEL_PIN, HIGH); // heater OFF
      }
      break;
    }

    case REFRACTORY: {
      if (now - modeTS >= REFRACTORY_MS) {
        sendText("STAT", "READY");
        sendText(nullptr, "");
        mode = IDLE;
      }
      break;
    }
  }
}
