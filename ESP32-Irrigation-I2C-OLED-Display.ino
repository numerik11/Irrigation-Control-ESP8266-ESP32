//i2c OLED

#ifndef ENABLE_DEBUG_ROUTES
  #define ENABLE_DEBUG_ROUTES 0   // set to 1 when you need them
#endif
#if ENABLE_DEBUG_ROUTES
  server.on("/i2c-test", HTTP_GET, handleI2CTest);
  server.on("/whereami", HTTP_GET, handleWhereAmI);
#endif
#ifndef ENABLE_OTA
  #define ENABLE_OTA 0
#endif
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LittleFS.h>
#include <PCF8574.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <esp_event.h>
#include <esp_system.h>
#include <math.h>
extern "C" {
  #include "esp_log.h"
}
#include <time.h>
#include <ESPmDNS.h> 
#include <PubSubClient.h>   // MQTT

// ---------- Hardware ----------
static const uint8_t MAX_ZONES = 16;

constexpr uint8_t I2C_SDA = 4;   //8 for s3
constexpr uint8_t I2C_SCL = 15;  //9 for s3
 
#ifndef STATUS_PIXEL_PIN
#define STATUS_PIXEL_PIN 48   // WS2812 status LED
#endif
static const uint8_t STATUS_PIXEL_COUNT = 1;

TwoWire I2Cbus = TwoWire(0);
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA, I2C_SCL);
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA, I2C_SCL);
Adafruit_NeoPixel statusPixel(STATUS_PIXEL_COUNT, STATUS_PIXEL_PIN, NEO_GRB + NEO_KHZ800);
bool statusPixelReady = false;

#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Cbus, OLED_RESET);

WiFiManager wifiManager;
WebServer server(80);
WiFiClient client;

// PCF mapping
const uint8_t ALL_P = 6;
const uint8_t PCH[ALL_P] = { P0, P1, P2, P3, P4, P5 };
uint8_t mainsChannel = P4; // reserved for mains relay
uint8_t tankChannel  = P5; // reserved for tank relay
uint8_t zonesCount   = 4;  // 1..16 

// GPIO fallback (configurable polarity)
bool useGpioFallback = false;
bool gpioActiveLow   = true;
bool relayActiveHigh = true;
bool tankEnabled     = true;

int zonePins[MAX_ZONES] = {
  18, 19, 12, 13, -1, -1,   // defaults: first 4 zones on PCF, extras via GPIO
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1   // extra slots up to 16
};
int mainsPin = 25;
int tankPin  = 26;

const int LED_PIN  = 4;
int tankLevelPin  = 11; // ADC input

// Physical rain sensor
bool rainSensorEnabled = false;
bool rainSensorInvert  = false;
int  rainSensorPin     = 27;

// Physical manual control buttons (disabled by default)
int manualSelectPin = -1;   // cycles the target zone (INPUT_PULLUP, -1 = disabled)
int manualStartPin  = -1;   // toggles start/stop for the selected zone (INPUT_PULLUP, -1 = disabled)
uint8_t manualSelectedZone = 0;
uint32_t manualScreenUntilMs = 0;
const unsigned long MANUAL_BTN_DEBOUNCE_MS = 60;

// ---------- Config / State ----------
String apiKey, city; // OpenWeather (city = city ID)
String cachedWeatherData;

// Weather cache / metrics
unsigned long lastWeatherUpdate = 0;
const unsigned long weatherUpdateInterval = 15UL * 60UL * 1000UL; // 15m

// Forecast cache / metrics
String cachedForecastData;
unsigned long lastForecastUpdate = 0;
const unsigned long forecastUpdateInterval = 60UL * 60UL * 1000UL; // 60m
float rainNext12h_mm = NAN;
float rainNext24h_mm = NAN;
int   popNext12h_pct = -1;
int   nextRainIn_h   = -1;
float maxGust24h_ms  = NAN;
float todayMin_C     = NAN, todayMax_C = NAN;
time_t todaySunrise  = 0,   todaySunset = 0;

// Delay controls
bool  rainDelayEnabled = true;
bool  windDelayEnabled = false;
bool  justUseTank = false;
bool  justUseMains = false;

// New saved features
bool     systemPaused = false;
uint32_t pauseUntilEpoch = 0;
bool     rainDelayFromForecastEnabled = true;  // gate for forecast-based rain

// NEW Master/Cooldown/Threshold
bool     systemMasterEnabled = true;     // Master On/Off
uint32_t rainCooldownUntilEpoch = 0;     // when > now => block starts
int      rainCooldownMin = 60;           // minutes to wait after rain clears
int      rainThreshold24h_mm = 5;        // forecast/actual 24h total triggers delay

// NEW Run mode: sequential (false) or concurrent (true)
bool runZonesConcurrent = false;

// Scheduling
bool enableStartTime2[MAX_ZONES] = {false};
bool days[MAX_ZONES][7] = {{false}};
bool zoneActive[MAX_ZONES] = {false};
bool pendingStart[MAX_ZONES] = {false};
int  windQueuedZone = -1;  // most recent scheduled zone delayed by wind
uint8_t lastStartSlot[MAX_ZONES] = {1}; // 1=primary, 2=secondary

bool     windActive = false;
bool     rainActive             = false;
bool     rainByWeatherActive    = false;
bool     rainBySensorActive     = false;
uint8_t  rainCooldownHours      = 24;      // or loaded from config


// legacy (kept for file compat; not used in logic)
float tzOffsetHours = 9.5f;

float windSpeedThreshold = 5.0f;
float lastRainAmount = 0.0f;
uint8_t tankLowThresholdPct = 10;

int startHour [MAX_ZONES] = {0};
int startMin  [MAX_ZONES] = {0};
int startHour2[MAX_ZONES] = {0};
int startMin2 [MAX_ZONES] = {0};
int durationMin[MAX_ZONES] = {0};
int durationSec[MAX_ZONES] = {0};
int duration2Min[MAX_ZONES] = {0};
int duration2Sec[MAX_ZONES] = {0};
int lastCheckedMinute[MAX_ZONES] = {
  -1,-1,-1,-1,-1,-1,-1,-1,
  -1,-1,-1,-1,-1,-1,-1,-1
};

int tankEmptyRaw = 100;
int tankFullRaw  = 900;
String zoneNames[MAX_ZONES] = {
  "Zone 1","Zone 2","Zone 3","Zone 4","Zone 5","Zone 6",
  "Zone 7","Zone 8","Zone 9","Zone 10","Zone 11","Zone 12",
  "Zone 13","Zone 14","Zone 15","Zone 16"
};

unsigned long zoneStartMs[MAX_ZONES] = {0};
unsigned long zoneRunTotalSec[MAX_ZONES] = {0}; // actual duration for the current run
unsigned long lastScreenRefresh = 0;

const uint8_t expanderAddrs[] = { 0x22, 0x24 };
const uint8_t I2C_HEALTH_DEBOUNCE = 10;
uint8_t i2cFailCount = 0;

// Debug
bool dbgForceRain = false;
bool dbgForceWind = false;

// Timing
static const uint32_t LOOP_SLEEP_MS    = 20;
static const uint32_t I2C_CHECK_MS     = 1000;
static const uint32_t TIME_QUERY_MS    = 1000;
static const uint32_t SCHEDULE_TICK_MS = 1000;
static uint32_t lastI2cCheck     = 0;
static uint32_t lastTimeQuery    = 0;
static uint32_t lastScheduleTick = 0;
static bool midnightDone = false;
static tm cachedTm = {};

// Uptime
uint32_t bootMillis = 0;

// ---------- NEW: Actual rainfall history (rolling 24h) + globals for 1h ----------
static float rainHist[24] = {0};   // last 24 hourly buckets (mm/hour)
static int   rainIdx = 0;          // points to most recent bucket
static time_t lastRainHistHour = 0;
static float last24hActualRain(); // forward
float rain1hNow = 0.0f;  // mm from /weather last 1h
float rain3hNow = 0.0f;  // mm from /weather last 3h

// ---------- NEW: guard to avoid fetching while serving HTTP ----------
volatile bool g_inHttp = false;
struct HttpScope {
  HttpScope()  { g_inHttp = true; }
  ~HttpScope() { g_inHttp = false; }
};

// ---------- Prototypes ----------
void wifiCheck();
void loadConfig();
void saveConfig();
void loadSchedule();
void saveSchedule();
void updateCachedWeather();
void tickWeather();                 // NEW
void HomeScreen();
void RainScreen();
void updateLCDForZone(int zone);
bool shouldStartZone(int zone);
bool hasDurationCompleted(int zone);
void turnOnZone(int zone);
void turnOffZone(int zone);
void turnOnValveManual(int z);
void turnOffValveManual(int z);
void handleRoot();
static void initExtraZoneGpios();
void handleSubmit();
void handleSetupPage();
void handleConfigure();
void handleLogPage();
void handleClearEvents();
void handleTankCalibration();
String fetchWeather();
String fetchForecast(float lat, float lon);
bool checkWindRain();
void checkI2CHealth();
void initGpioFallback();
bool initExpanders();
void toggleBacklight();
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed);
void printCurrentTime();
int    tankPercent();
bool   isTankLow();
String sourceModeText();
void initManualButtons();
void tickManualButtons();
void showManualSelection();
void drawManualSelection();
void statusPixelSet(uint8_t r,uint8_t g,uint8_t b);
void updateStatusPixel();


// ===================== Timezone config =====================
enum TZMode : uint8_t { TZ_POSIX = 0, TZ_IANA = 1, TZ_FIXED = 2 };
TZMode tzMode = TZ_POSIX;
String tzPosix = "ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3"; // default
String tzIANA  = "Australia/Adelaide";
int16_t tzFixedOffsetMin = 570;

static void applyTimezoneAndSNTP() {
  const char* ntp1 = "pool.ntp.org";
  const char* ntp2 = "time.google.com";
  const char* ntp3 = "time.cloudflare.com";

  switch (tzMode) {
    case TZ_IANA: configTzTime(tzIANA.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_POSIX: configTzTime(tzPosix.c_str(), ntp1, ntp2, ntp3); break;
    case TZ_FIXED: {
      long offSec = (long)tzFixedOffsetMin * 60L;
      configTime(offSec, 0, ntp1, ntp2, ntp3);
      int m = tzFixedOffsetMin; int sign = (m >= 0) ? -1 : 1; m = abs(m);
      int hh = m/60, mm = m%60; char buf[32];
      snprintf(buf, sizeof(buf), "GMT%+d:%02d", sign*hh, sign*mm);
      setenv("TZ", buf, 1); tzset(); break;
    }
  }
  time_t now = time(nullptr);
  for (int i=0; i<50 && now < 1000000000; ++i) { delay(200); now = time(nullptr); }
}

// ---------- MQTT ----------
bool   mqttEnabled = false;
String mqttBroker  = "";
uint16_t mqttPort  = 1883;
String mqttUser    = "";
String mqttPass    = "";
String mqttBase    = "espirrigation";

WiFiClient   _mqttNetCli;
PubSubClient _mqtt(_mqttNetCli);
uint32_t     _lastMqttPub = 0;

void mqttSetup(){
  if (!mqttEnabled || mqttBroker.length()==0) return;
  _mqtt.setServer(mqttBroker.c_str(), mqttPort);
  _mqtt.setCallback([](char* topic, byte* payload, unsigned int len){
    String t(topic), msg; msg.reserve(len);
    for (unsigned i=0;i<len;i++) msg += (char)payload[i];

    if (t.endsWith("/cmd/master")) {
      systemMasterEnabled = (msg=="on"||msg=="ON"||msg=="1");
      saveConfig();
    } else if (t.endsWith("/cmd/pause")) {
      uint32_t sec = msg.toInt();
      systemPaused = true; pauseUntilEpoch = sec ? (time(nullptr)+sec) : 0; saveConfig();
    } else if (t.endsWith("/cmd/resume")) {
      systemPaused=false; pauseUntilEpoch=0; saveConfig();
    } else if (t.endsWith("/cmd/stop_all")) {
      for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    } else if (t.indexOf("/cmd/zone/")!=-1) {
      int p=t.lastIndexOf('/'); int z=(p>=0)? t.substring(p+1).toInt():-1;
      if (z>=0 && z<(int)zonesCount){
        if (msg=="on"||msg=="ON"||msg=="1") turnOnValveManual(z);
        else                                turnOffValveManual(z);
      }
    }
  });
}
void mqttEnsureConnected(){
  if (!mqttEnabled) return;
  if (_mqtt.connected()) return;
  String cid = "espirrigation-" + WiFi.macAddress();
  if (_mqtt.connect(cid.c_str(),
      mqttUser.length()?mqttUser.c_str():nullptr,
      mqttPass.length()?mqttPass.c_str():nullptr)) {
    _mqtt.subscribe( (mqttBase + "/cmd/#").c_str() );
  }
}

void mqttPublishStatus(){
  if (!mqttEnabled || !_mqtt.connected()) return;
  if (millis() - _lastMqttPub < 3000) return;
  _lastMqttPub = millis();

  DynamicJsonDocument d(1024 + MAX_ZONES*64);
  d["masterOn"] = systemMasterEnabled;
  d["paused"]   = (systemPaused && (pauseUntilEpoch==0 || time(nullptr)<(time_t)pauseUntilEpoch));
  d["cooldownRemaining"] = (rainCooldownUntilEpoch>time(nullptr)? (rainCooldownUntilEpoch - time(nullptr)) : 0);
  d["rainActive"] = rainActive;
  d["windActive"] = windActive;
  d["tankPct"]    = tankPercent();
  d["sourceMode"] = sourceModeText();
  d["rain24hActual"] = last24hActualRain();  // NEW
  d["runConcurrent"] = runZonesConcurrent;   // NEW
  JsonArray arr = d.createNestedArray("zones");
  for (int i=0;i<zonesCount;i++){
    JsonObject z = arr.createNestedObject();
    z["name"] = zoneNames[i];
    z["active"] = zoneActive[i];
  }
  String out; serializeJson(d,out);
  _mqtt.publish( (mqttBase + "/status").c_str(), out.c_str(), true);
}

// ---------- Helpers ----------
static inline int i_min(int a, int b) { return (a < b) ? a : b; }

static inline bool isPausedNow() {
  time_t now = time(nullptr);
  return systemPaused && (pauseUntilEpoch == 0 || now < (time_t)pauseUntilEpoch);
}

static inline bool isBlockedNow(){
  if (!systemMasterEnabled) return true;
  if (isPausedNow()) return true;
  time_t now = time(nullptr);
  if (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch) return true;
  return false;
}

// --- Cancel helper: cancel a (would-be) start instead of queueing ---
static inline void cancelStart(int z, const char* reason, bool dueToRain) {
  if (z >= 0 && z < (int)MAX_ZONES) pendingStart[z] = false; // ensure not queued
  logEvent(z, "CANCELLED", reason, dueToRain);               // record why it was cancelled
}

static bool i2cPing(uint8_t addr) {
  I2Cbus.beginTransmission(addr);
  return (I2Cbus.endTransmission() == 0);
}

static String cleanName(String s) {
  s.trim(); s.replace("\r",""); s.replace("\n","");
  if (s.length() > 32) s = s.substring(0,32);
  return s;
}

inline bool isValidAdcPin(int pin) {
  // Generic guard for ESP32 family (0..39 are GPIO-capable)
  return (pin >= 0 && pin <= 39);
}

int tankPercent() {
  if (!isValidAdcPin(tankLevelPin)) {
    static bool warned = false;
    if (!warned) {
      Serial.printf("[TANK] Invalid ADC pin %d.\n", tankLevelPin);
      warned = true;
    }
    return 0;
  }
  const int N=8; uint32_t acc=0;
  for (int i=0;i<N;i++){ acc += analogRead(tankLevelPin); delayMicroseconds(200); }
  int raw=acc/N;
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isTankLow() {
  if (!tankEnabled) return true; // disabled => force mains
  return tankPercent() <= tankLowThresholdPct;
}

String sourceModeText() {
  if (!tankEnabled) return "TankDisabled";
  if (justUseTank)  return "Force:Tank";
  if (justUseMains) return "Force:Mains";
  return isTankLow() ? "Auto:Mains" : "Auto:Tank";
}

static float chipTemperatureC() {
  // Arduino core provides temperatureRead() for ESP32 family; clamp obvious bad readings
  float t = temperatureRead();
  if (isnan(t) || t < -40.0f || t > 150.0f) return NAN;
  return t;
}

static inline void chooseWaterSource(const char*& src, bool& mainsOn, bool& tankOn) {
  mainsOn = false; tankOn = false;
  if (!tankEnabled) { src = "TankDisabled"; mainsOn = true; return; }
  if (justUseTank)  { src = "Tank";  tankOn  = true; return; }
  if (justUseMains) { src = "Mains"; mainsOn = true; return; }
  if (isTankLow())  { src = "Mains"; mainsOn = true; return; }
  src = "Tank"; tankOn = true;
}

void statusPixelSet(uint8_t r,uint8_t g,uint8_t b) {
  if (!statusPixelReady) return;
  statusPixel.setPixelColor(0, statusPixel.Color(r,g,b));
  statusPixel.show();
}

void updateStatusPixel() {
  if (!statusPixelReady) return;

  // Flash blue when in AP/config-portal mode to signal setup state.
  bool inApMode = (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA || wifiManager.getConfigPortalActive());
  if (inApMode) {
    static uint32_t lastBlink = 0;
    static bool on = false;
    uint32_t now = millis();
    if (now - lastBlink >= 400) { on = !on; lastBlink = now; }
    if (on) statusPixelSet(0, 0, 32); else statusPixelSet(0, 0, 0);
    return;
  }

  // Default: OK (green)
  uint8_t r = 0, g = 20, b = 0;

  // Is any zone currently active?
  bool anyZoneOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyZoneOn = true; break; }
  }

  if (WiFi.status() != WL_CONNECTED) {
    // Wi-Fi not connected yet: magenta
    r = 20; g = 0; b = 12;
  } else if (anyZoneOn) {
    // Watering in progress: Blue
    r = 0; g = 0; b = 24;
  } else if (!systemMasterEnabled || isPausedNow()) {
    // Master off or paused: red
    r = 24; g = 0; b = 0;
  } else if (rainActive || windActive || isBlockedNow()) {
    // Weather/cooldown blocking: amber
    r = 24; g = 12; b = 0;
  } else if (useGpioFallback) {
    // Fallback mode: Green
    r = 0; g = 60; b = 0;
  }

  statusPixelSet(r,g,b);
}

bool physicalRainNowRaw() {
  if (!rainSensorEnabled) return false;
  pinMode(rainSensorPin, INPUT_PULLUP);
  int v = digitalRead(rainSensorPin); // LOW=dry, HIGH=wet (NC default)
  bool wet = (v == HIGH);
  if (rainSensorInvert) wet = !wet;
  return wet;
}

String rainDelayCauseText() {
  if (!rainDelayEnabled) return "Disabled";

  time_t now = time(nullptr);

  // Not currently raining, check for active cooldown with countdown
  if (!rainActive) {
    if (rainCooldownUntilEpoch && now < (time_t)rainCooldownUntilEpoch) {
      uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - now); // seconds left
      // Round up to minutes so "59s" still shows as "1m"
      uint32_t mins = (rem + 59U) / 60U;

      char buf[40];

      if (mins >= 60U) {
        uint32_t h = mins / 60U;
        uint32_t m = mins % 60U;
        if (m > 0U) {
          // Example: "Cooldown 1h 05m"
          snprintf(buf, sizeof(buf), "Cooldown %luh %lum",
                   (unsigned long)h, (unsigned long)m);
        } else {
          // Example: "Cooldown 2h"
          snprintf(buf, sizeof(buf), "Cooldown %luh",
                   (unsigned long)h);
        }
      } else {
        // Under 1 hour: "Cooldown 23m"
        snprintf(buf, sizeof(buf), "Cooldown %lum",
                 (unsigned long)mins);
      }

      return String(buf);
    }

    if (!systemMasterEnabled) return "Master Off";
    if (isPausedNow())        return "Paused";
    if (windActive)           return "Windy";
    return "None";
  }

  // Currently in a rain-delay state
  if (rainByWeatherActive && rainBySensorActive) return "Both";
  if (rainByWeatherActive) return "Raining";
  if (rainBySensorActive)  return "Sensor Wet";
  return "Active";
}

static const char* kHost = "espirrigation";
static const wifi_power_t kWifiTxPower = WIFI_POWER_19_5dBm;

static void mdnsStart() {
  MDNS.end(); // in case it was running
  if (MDNS.begin(kHost)) {
    MDNS.addService("_http", "_tcp", 80);
    Serial.println("[mDNS] started: http://espirrigation.local/");
  } else {
    Serial.println("[mDNS] begin() failed");
  }
}

static const char* resetReasonText(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "external reset";
    case ESP_RST_SW:        return "software reset";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "interrupt watchdog";
    case ESP_RST_TASK_WDT:  return "task watchdog";
    case ESP_RST_WDT:       return "other watchdog";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "unknown";
  }
}

// ---------- GPIO fallback helpers ----------
inline int gpioLevel(bool on) {
  // on = true -> valve ON
  // on = false -> valve OFF
  if (gpioActiveLow) {
    // Active-LOW: LOW = ON, HIGH = OFF
    return on ? LOW : HIGH;
  } else {
    // Active-HIGH: HIGH = ON, LOW = OFF
    return on ? HIGH : LOW;
  }
}

inline void gpioInitOutput(int pin) {
  if (pin < 0 || pin > 39) return;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, gpioLevel(false));  // ensure OFF
}

inline void gpioZoneWrite(int z, bool on) {
  if (z < 0 || z >= (int)MAX_ZONES) return;
  int pin = zonePins[z];
  if (pin < 0 || pin > 39) return;
  digitalWrite(pin, gpioLevel(on));
}

inline void gpioSourceWrite(bool mainsOn, bool tankOn) {
  if (mainsPin >= 0 && mainsPin <= 39) digitalWrite(mainsPin, gpioLevel(mainsOn));
  if (tankPin  >= 0 && tankPin  <= 39) digitalWrite(tankPin,  gpioLevel(tankOn));
}

inline void setWaterSourceRelays(bool mainsOn, bool tankOn) {
  if (!useGpioFallback) {
    // PCF8574: active-LOW
    pcfOut.digitalWrite(mainsChannel, mainsOn ? LOW : HIGH);
    pcfOut.digitalWrite(tankChannel,  tankOn  ? LOW : HIGH);
  } else {
    gpioSourceWrite(mainsOn, tankOn);
  }
}

inline bool useExpanderForZone(int z) {
  if (useGpioFallback) return false;
  // Use PCF for zones 0-3 only; keep P4/P5 free for mains/tank relays
  if (z < 0 || z >= 4) return false;
  return true;
}

// Duration helper: slot=1 (primary) or 2 (secondary) with fallback to primary
static inline unsigned long durationForSlot(int z, int slot) {
  if (z < 0 || z >= (int)MAX_ZONES) return 0;
  if (slot == 2 && enableStartTime2[z]) {
    unsigned long m = (duration2Min[z] >= 0) ? (unsigned long)duration2Min[z] : 0;
    unsigned long s = (duration2Sec[z] >= 0) ? (unsigned long)duration2Sec[z] : 0;
    unsigned long tot = m * 60UL + s;
    if (tot > 0) return tot;
  }
  unsigned long m = (durationMin[z] >= 0) ? (unsigned long)durationMin[z] : 0;
  unsigned long s = (durationSec[z] >= 0) ? (unsigned long)durationSec[z] : 0;
  return m * 60UL + s;
}


// ---------- Next Water type + forward decl ----------
struct NextWaterInfo {
  time_t   epoch;    // local epoch for the next start
  int      zone;     // zone index
  uint32_t durSec;   // duration in seconds
};
static NextWaterInfo computeNextWatering();

// ---------- I2C init ----------
bool initExpanders() {
  bool haveIn  = i2cPing(0x22);
  bool haveOut = i2cPing(0x24);
  Serial.printf("[I2C] ping 0x22=%d 0x24=%d\n", haveIn, haveOut);
  if (!haveOut) return false;

  for (int i=0;i<3 && !pcfOut.begin();++i) delay(5);
  for (int i=0;i<3 && !pcfIn.begin(); ++i) delay(5);

  for (uint8_t ch=P0; ch<=P5; ch++) { pcfOut.pinMode(ch, OUTPUT); pcfOut.digitalWrite(ch, HIGH); }
  for (uint8_t ch=P0; ch<=P5; ch++) { pcfIn.pinMode (ch, INPUT);  pcfIn.digitalWrite(ch, HIGH); }
  return true;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Serial.printf("[BOOT] reset reason=%d (%s)\n",
                (int)esp_reset_reason(), resetReasonText(esp_reset_reason()));

  // Clamp noisy logs (IDF)
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("i2c", ESP_LOG_NONE);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c_master", ESP_LOG_NONE);

  // I2C bus
  I2Cbus.begin(I2C_SDA, I2C_SCL, 100000);
  I2Cbus.setTimeOut(20);

  bootMillis = millis();

  // LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed; formatting...");
    if (!(LittleFS.format() && LittleFS.begin())) {
      Serial.println("LittleFS unavailable; halt.");
      while (true) delay(1000);
    }
  }

  // Config + schedule
  loadConfig();
  if (!LittleFS.exists("/schedule.txt")) saveSchedule();
  loadSchedule();
  initManualButtons();

  mainsChannel = P4; 
  tankChannel  = P5;

  // PCF8574 expanders (or GPIO fallback)
  if (!initExpanders()) {
    Serial.println("PCF8574 relays not found; GPIO fallback.");
    initGpioFallback();
    useGpioFallback = true;
  } else {
    useGpioFallback = false;
    initExtraZoneGpios();  // prepare GPIO pins for zones >=4 while using PCF for zones 0-3
    checkI2CHealth();
  }

  pinMode(LED_PIN, OUTPUT);

  // Status pixel (WS2812)
  if (STATUS_PIXEL_PIN >= 0) {
    statusPixel.begin();
    statusPixel.setBrightness(32);
    statusPixel.clear();
    statusPixel.show();
    statusPixelReady = true;
    statusPixelSet(0, 0, 20); // boot = blue
  } else {
    statusPixelReady = false;
  }

  // OLED boot splash
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2); display.setCursor(0, 8);  display.print("Irrigation");
  display.setTextSize(1); display.setCursor(0, 36); display.print("AP: ESPIrrigationAP");
  display.setCursor(0, 50); display.print("http://192.168.4.1");
  display.display();

  // Hostname + WiFi events
  WiFi.setHostname(kHost);
  WiFi.persistent(true);           // keep creds in NVS across resets
  WiFi.setAutoReconnect(true);

  // WiFiManager connect (prefer fast STA connect; fall back to portal with timeout)
  wifiManager.setWiFiAutoReconnect(true); // extra nudge for ESP32(S3)
  wifiManager.setConnectRetries(6);       // give STA more chances before portal
  wifiManager.setConnectTimeout(20);      // seconds per retry
  wifiManager.setSaveConnect(true);       // reconnect immediately after saving
  wifiManager.setTimeout(45);             // stop trying after 45s
  wifiManager.setConfigPortalTimeout(120);   // portal stays up max 2 minutes
  wifiManager.setBreakAfterConfig(true);     // exit once credentials are saved
  WiFi.mode(WIFI_STA);                        // ensure STA mode for S2/S3
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    Serial.println("WiFi connect failed -> opening config portal (ESPIrrigationAP).");
    wifiManager.startConfigPortal("ESPIrrigationAP");  // limited-time portal
  }

  // Improve HTTP responsiveness
  WiFi.setTxPower(kWifiTxPower);
  WiFi.setSleep(false); // NEW: disable modem sleep for snappier responses

  // Connected screen
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(0, 0);  display.print("Connected!");
  display.setTextSize(1); display.setCursor(0, 20); display.print(WiFi.localIP().toString());
  display.setCursor(0, 32); display.print("espirrigation.local");
  display.display();
  delay(2500);

  // Timezone + SNTP
  delay(250);
  applyTimezoneAndSNTP();
  {
    time_t now = time(nullptr);
    struct tm tcheck; localtime_r(&now, &tcheck);
    Serial.printf("[TIME] NTP ok: %04d-%02d-%02d %02d:%02d:%02d tz=%s mode=%d\n",
      tcheck.tm_year+1900, tcheck.tm_mon+1, tcheck.tm_mday,
      tcheck.tm_hour, tcheck.tm_min, tcheck.tm_sec,
      (tcheck.tm_isdst>0) ? "DST" : "STD", (int)tzMode);
  }

  // OTA
  #if ENABLE_OTA
    ArduinoOTA.setHostname("ESP32-Irrigation-OTA");
    ArduinoOTA.begin();
  #endif

  mdnsStart();

  // -------- Routes --------
  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);

  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);

  server.on("/events", HTTP_GET, handleLogPage);
  server.on("/clearevents", HTTP_POST, handleClearEvents);

  server.on("/tank", HTTP_GET, handleTankCalibration);

  // /status JSON
  server.on("/status", HTTP_GET, [](){
    HttpScope _scope; // NEW: avoid heavy work while serving
    DynamicJsonDocument doc(4096 + MAX_ZONES * 160);

    // DO NOT fetch here anymore
    // updateCachedWeather();

    doc["rainDelayActive"] = rainActive;
    doc["windDelayActive"] = windActive;
    doc["rainDelayCause"]  = rainDelayCauseText();
    doc["zonesCount"]      = zonesCount;
    doc["tankPct"]         = tankPercent();
    doc["sourceMode"]      = sourceModeText();
    doc["rssi"]            = WiFi.RSSI();
    float chipTempC        = chipTemperatureC();
    doc["chipTempC"]       = isnan(chipTempC) ? 0.0f : chipTempC;
    doc["chipTempValid"]   = !isnan(chipTempC);
    doc["uptimeSec"]       = (millis() - bootMillis) / 1000;

    // Current rain (actuals)
    doc["rain1hNow"] = rain1hNow;
    doc["rain3hNow"] = rain3hNow;

    // Forecast fields
    doc["rain12h"]     = isnan(rainNext12h_mm) ? 0.0f : rainNext12h_mm;
    doc["rain24h"]     = isnan(rainNext24h_mm) ? 0.0f : rainNext24h_mm;
    doc["pop12h"]      = (popNext12h_pct < 0 ? 0 : popNext12h_pct);
    doc["nextRainInH"] = (nextRainIn_h < 0 ? 255 : nextRainIn_h);
    doc["gust24h"]     = isnan(maxGust24h_ms) ? 0.0f : maxGust24h_ms;
    doc["tmin"]        = isnan(todayMin_C) ? 0.0f : todayMin_C;
    doc["tmax"]        = isnan(todayMax_C) ? 0.0f : todayMax_C;
    doc["sunrise"]     = (uint32_t)todaySunrise;
    doc["sunset"]      = (uint32_t)todaySunset;

    // local vs UTC offset
    time_t nowEpoch = time(nullptr);
    struct tm ltm; localtime_r(&nowEpoch, &ltm);
    struct tm gtm; gmtime_r(&nowEpoch,  &gtm);
    int localMin = ltm.tm_hour*60 + ltm.tm_min;
    int utcMin   = gtm.tm_hour*60 + gtm.tm_min;
    int deltaMin = localMin - utcMin;
    if (deltaMin >  12*60) deltaMin -= 24*60;
    if (deltaMin < -12*60) deltaMin += 24*60;

    auto hhmm = [](time_t t){
      char b[6]; struct tm tt; localtime_r(&t,&tt);
      strftime(b,sizeof(b),"%H:%M",&tt); return String(b);
    };

    doc["deviceEpoch"]  = (uint32_t)nowEpoch;
    doc["utcOffsetMin"] = deltaMin;
    doc["isDST"]        = (ltm.tm_isdst > 0);
    doc["tzAbbrev"]     = (ltm.tm_isdst>0) ? "DST" : "STD";
    doc["sunriseLocal"] = hhmm((time_t)todaySunrise);
    doc["sunsetLocal"]  = hhmm((time_t)todaySunset);

    // Feature gates
    doc["masterOn"]          = systemMasterEnabled;
    doc["cooldownUntil"]     = rainCooldownUntilEpoch;
    doc["cooldownRemaining"] = (rainCooldownUntilEpoch>nowEpoch) ? (rainCooldownUntilEpoch - nowEpoch) : 0;
    doc["rainThresh24h"]     = rainThreshold24h_mm;
    doc["rainCooldownMin"]   = rainCooldownMin;
    doc["rainCooldownHours"] = rainCooldownMin / 60;

    // NEW: expose run mode
    doc["runConcurrent"] = runZonesConcurrent;

    // Zones snapshot
    JsonArray zones = doc.createNestedArray("zones");
    for (int i=0; i<zonesCount; i++){
      JsonObject z = zones.createNestedObject();
      z["active"] = zoneActive[i];
      z["name"]   = zoneNames[i];
      unsigned long rem = 0;
      unsigned long total = zoneRunTotalSec[i];
      if (total == 0) total = durationForSlot(i,1);
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
        rem = (elapsed < total ? total - elapsed : 0);
      }
      z["remaining"] = rem;
      z["totalSec"]  = total;
    }

    // Current weather pass-through (no fetch, just decode cache)
    {
      DynamicJsonDocument js(2048);
      if (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok) {
        doc["temp"]       = js["main"]["temp"]       | 0.0f;
        doc["feels_like"] = js["main"]["feels_like"] | 0.0f;
        doc["humidity"]   = js["main"]["humidity"]   | 0;
        doc["pressure"]   = js["main"]["pressure"]   | 0;
        doc["wind"]       = js["wind"]["speed"]      | 0.0f;
        doc["gustNow"]    = js["wind"]["gust"]       | 0.0f;
        doc["condMain"]   = js["weather"][0]["main"]        | "";
        doc["condDesc"]   = js["weather"][0]["description"] | "";
        doc["icon"]       = js["weather"][0]["icon"]        | "";
        doc["cityName"]   = js["name"]                      | "";
        doc["owmTzSec"]   = js["timezone"]                  | 0;
      }
    }

    // Next Water (queue-first)
    {
      NextWaterInfo nw = computeNextWatering();
      if (nw.zone >= 0) {
        doc["nextWaterEpoch"]  = (uint32_t)nw.epoch;
        doc["nextWaterZone"]   = nw.zone;
        doc["nextWaterName"]   = zoneNames[nw.zone];
        doc["nextWaterDurSec"] = nw.durSec;
      } else {
        doc["nextWaterEpoch"]  = 0;
        doc["nextWaterZone"]   = 255;
        doc["nextWaterName"]   = "";
        doc["nextWaterDurSec"] = 0;
      }
    }

    // Pause / delay toggles
    doc["systemPaused"]          = isPausedNow();
    doc["pauseUntil"]            = pauseUntilEpoch;
    doc["rainForecastEnabled"]   = rainDelayFromForecastEnabled;
    doc["rainSensorEnabled"]     = rainSensorEnabled;

    // NEW: actual rolling 24h rainfall
    doc["rain24hActual"] = last24hActualRain();

    String out; serializeJson(doc, out);
    server.send(200, "application/json", out);
  });

  // Time API
  server.on("/api/time", HTTP_GET, [](){
    HttpScope _scope;
    time_t nowEpoch = time(nullptr);
    struct tm lt; localtime_r(&nowEpoch, &lt);
    struct tm gt; gmtime_r(&nowEpoch,  &gt);
    DynamicJsonDocument d(512);
    d["epoch"] = (uint32_t)nowEpoch;
    char buf[32];
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&lt); d["local"] = buf;
    strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&gt); d["utc"]   = buf;
    d["isDST"] = (lt.tm_isdst>0);
    d["tz"]    = (lt.tm_isdst>0) ? "DST" : "STD";
    String out; serializeJson(d,out);
    server.send(200,"application/json",out);
  });

  // Tank calibration endpoints
  server.on("/setTankEmpty", HTTP_POST, []() {
    HttpScope _scope;
    tankEmptyRaw = analogRead(tankLevelPin);
    saveConfig();
    server.sendHeader("Location", "/tank", true);
    server.send(302, "text/plain", "");
  });
  server.on("/setTankFull", HTTP_POST, []() {
    HttpScope _scope;
    tankFullRaw = analogRead(tankLevelPin);
    saveConfig();
    server.sendHeader("Location", "/tank", true);
    server.send(302, "text/plain", "");
  });

  // Manual control per zone
  for (int i=0; i<MAX_ZONES; i++){
    server.on(String("/valve/on/")+i,  HTTP_POST, [i](){ HttpScope _s; turnOnValveManual(i);  server.send(200,"text/plain","OK"); });
    server.on(String("/valve/off/")+i, HTTP_POST, [i](){ HttpScope _s; turnOffValveManual(i); server.send(200,"text/plain","OK"); });
  }
  server.on("/stopall", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<MAX_ZONES; ++z) if (zoneActive[z]) turnOffZone(z);
    server.send(200,"text/plain","OK");
  });
  server.on("/toggleBacklight", HTTP_POST, [](){
    HttpScope _scope;
    static bool inverted=false; inverted=!inverted;
    display.invertDisplay(inverted);
    server.send(200,"text/plain","Backlight toggled");
  });

  // I2C tools
  server.on("/i2c-test", HTTP_GET, [](){
    HttpScope _scope;
    if (useGpioFallback) { server.send(500,"text/plain","Fallback active"); return; }
    for (uint8_t ch : PCH) { pcfOut.digitalWrite(ch, LOW); delay(100); pcfOut.digitalWrite(ch, HIGH); delay(60); }
    server.send(200,"text/plain","PCF8574 pulse OK");
  });
  
  // Downloads / Admin
  server.on("/download/config.txt", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/config.txt")){ File f=LittleFS.open("/config.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/schedule.txt", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/schedule.txt")){ File f=LittleFS.open("/schedule.txt","r"); server.streamFile(f,"text/plain"); f.close(); }
    else server.send(404,"text/plain","missing");
  });
  server.on("/download/events.csv", HTTP_GET, [](){
    HttpScope _scope;
    if (LittleFS.exists("/events.csv")){ File f=LittleFS.open("/events.csv","r"); server.streamFile(f,"text/csv"); f.close(); }
    else server.send(404,"text/plain","No event log");
  });
  server.on("/reboot", HTTP_POST, [](){
    HttpScope _scope;
    server.send(200,"text/plain","restarting"); delay(200); ESP.restart();
  }); 
  // Pause/Resume/Delays/Forecast toggle
  server.on("/clear_delays", HTTP_POST, [](){
    HttpScope _scope;
    for (int z=0; z<(int)zonesCount; ++z) pendingStart[z] = false;
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    dbgForceRain = false; dbgForceWind = false;
    for (int z=0; z<(int)zonesCount; ++z) lastCheckedMinute[z] = -1;
    server.send(200,"text/plain","OK");
  });
  server.on("/pause", HTTP_POST, [](){
    HttpScope _scope;
    time_t nowEp = time(nullptr);
    String secStr = server.arg("sec");
    uint32_t sec = secStr.length()? secStr.toInt() : (24u*3600u);
    systemPaused = true;
    pauseUntilEpoch = sec ? (nowEp + sec) : 0;
    saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/resume", HTTP_POST, [](){
    HttpScope _scope;
    systemPaused = false; pauseUntilEpoch = 0; saveConfig();
    server.send(200,"text/plain","OK");
  });
  server.on("/set_rain_forecast", HTTP_POST, [](){
    HttpScope _scope;
    rainDelayFromForecastEnabled = server.hasArg("on");
    saveConfig();
    server.send(200,"text/plain", rainDelayFromForecastEnabled ? "on" : "off");
  });

  // Master and cooldown
  server.on("/master", HTTP_POST, [](){
    HttpScope _scope;
    systemMasterEnabled = server.hasArg("on");  // Dashboard only (not in Setup)
    saveConfig();
    server.send(200,"text/plain", systemMasterEnabled ? "on" : "off");
  });
  server.on("/clear_cooldown", HTTP_POST, [](){
    HttpScope _scope;
    rainCooldownUntilEpoch = 0;
    server.send(200,"text/plain","OK");
  });

  server.begin();

  // MQTT
  mqttSetup();
  mqttEnsureConnected();
}

// ---------- Loop ----------
void loop() {
  const uint32_t now = millis();
  #if ENABLE_OTA
  ArduinoOTA.handle();
  #endif

  // Service HTTP first (keeps UI responsive), then background tasks
  server.handleClient();
  wifiCheck();
  checkWindRain();
  mqttEnsureConnected();
  mqttPublishStatus();
  tickWeather();          // timed weather fetch (non-blocking)
  tickManualButtons();

  // Hard block while paused/master off/cooldown
  if (isBlockedNow()) {
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); }
    delay(15); return;
  }

  bool delayActive = false;
  if (rainActive || windActive) {
    delayActive = true;
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); }
    delay(15);
  }

  if (now - lastTimeQuery >= TIME_QUERY_MS) {
    lastTimeQuery = now;
    time_t nowTime = time(nullptr);
    localtime_r(&nowTime, &cachedTm);
  }

  if (cachedTm.tm_hour == 0 && cachedTm.tm_min == 0) {
    if (!midnightDone) {
      memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
      todayMin_C = NAN;
      todayMax_C = NAN;
      midnightDone = true;
    }
  } else midnightDone = false;

  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now; checkI2CHealth();
  }

  // WS2812 status pixel (non-blocking, light refresh)
  static uint32_t lastPixelUpdate = 0;
  if (statusPixelReady && now - lastPixelUpdate >= 500) {
    lastPixelUpdate = now;
    updateStatusPixel();
  }

  bool anyActive=false;
  for (int z=0; z<(int)zonesCount; z++) if (zoneActive[z]) { anyActive=true; break; }

  // -------- SCHEDULER (supports sequential or concurrent) --------
  if (now - lastScheduleTick >= SCHEDULE_TICK_MS) {
    lastScheduleTick = now;

    if (!isBlockedNow()) {
      for (int z=0; z<(int)zonesCount; z++) {
        if (shouldStartZone(z)) {
          // Cancel when blocked/rain; queue during wind so it can run later.
          if (isBlockedNow()) { cancelStart(z, "BLOCKED", false); continue; }
          if (rainActive)     { cancelStart(z, "RAIN",    true ); continue; }
          if (windActive)     { windQueuedZone = z; logEvent(z,"QUEUED","WIND",false); continue; }

          if (!runZonesConcurrent) {
            // sequential: only start if nothing is running; else queue (still allowed)
            if (anyActive) { pendingStart[z] = true; logEvent(z,"QUEUED","ACTIVE RUN",false); }
            else { turnOnZone(z); anyActive = true; }
          } else {
            // concurrent: start regardless of other active zones
            turnOnZone(z); anyActive = true;
          }
        }
        if (zoneActive[z] && hasDurationCompleted(z)) turnOffZone(z);
      }

      // Wind-delayed: start only the most recent scheduled zone when clear
      if (!rainActive && !windActive && windQueuedZone >= 0) {
        int wz = windQueuedZone;
        windQueuedZone = -1;
        if (!runZonesConcurrent && anyActive) {
          pendingStart[wz] = true;
          logEvent(wz, "QUEUED", "ACTIVE RUN", false);
        } else {
          turnOnZone(wz);
          anyActive = true;
        }
      }

      if (!runZonesConcurrent) {
        // sequential: drain one queued zone if nothing currently running
        if (!anyActive && !rainActive && !windActive) {
          for (int z=0; z<(int)zonesCount; z++) if (pendingStart[z]) { pendingStart[z]=false; turnOnZone(z); break; }
        }
      } else if (!rainActive && !windActive) {
        // concurrent: start any queued zones once delays clear
        for (int z=0; z<(int)zonesCount; z++) {
          if (pendingStart[z]) { pendingStart[z]=false; turnOnZone(z); anyActive = true; }
        }
      }
      // concurrent: queued starts (due to delays/blocks) will fire when delays clear
    }
  }

  if (now - lastScreenRefresh >= 1000) {
    lastScreenRefresh = now;
    if (delayActive) {
      RainScreen();
    } else if (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - now) > 0) {
      drawManualSelection();
    } else {
      manualScreenUntilMs = 0;
      HomeScreen();
    }
  }
  delay(LOOP_SLEEP_MS);
}

// ---------- Connectivity / I2C health ----------
void wifiCheck() {
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect(false, false);   // do NOT erase saved credentials
    WiFi.reconnect();

    // wait up to ~8s for reconnect using stored creds (no portal)
    unsigned long t0 = millis();
    while (WiFi.status()!=WL_CONNECTED && (millis() - t0) < 8000UL) {
      delay(250);
    }

    if (WiFi.status()==WL_CONNECTED) {
      Serial.println("Reconnected.");
      WiFi.setSleep(false); // keep disabled after reconnect
      WiFi.setTxPower(kWifiTxPower);
      WiFi.enableLongRange(true);
    } else {
      Serial.println("Reconnection failed (kept creds, not opening portal).");
    }
  }
}

void checkI2CHealth() {
  delay(20);
  bool anyErr=false;
  for (uint8_t addr : expanderAddrs) {
    I2Cbus.beginTransmission(addr);
    if (I2Cbus.endTransmission()!=0) { anyErr=true; break; }
  }
  if (anyErr) {
    i2cFailCount++;
    if (i2cFailCount >= I2C_HEALTH_DEBOUNCE) {
      Serial.println("I2C unstable -> GPIO fallback");
      useGpioFallback = true; initGpioFallback();
    }
  } else i2cFailCount = 0;
}

void initGpioFallback() {
  useGpioFallback = true;

  for (uint8_t i = 0; i < MAX_ZONES; i++) {
    gpioInitOutput(zonePins[i]);    // OFF based on gpioActiveLow
  }

  gpioInitOutput(mainsPin);         // OFF
  gpioInitOutput(tankPin);          // OFF
}

// Initialize GPIO pins for zones that are not on the PCF expander (z >= 4)
static void initExtraZoneGpios() {
  for (uint8_t z = 4; z < MAX_ZONES; z++) {
    gpioInitOutput(zonePins[z]);
  }
}

// ---------- Manual hardware buttons (select + start/stop) ----------
void showManualSelection() {
  manualScreenUntilMs = millis() + 15000UL;
  drawManualSelection();
}

void drawManualSelection() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("Manual");
  display.setCursor(0, 24);
  display.print("Zone ");
  display.print((int)manualSelectedZone + 1);
  display.display();
}

void initManualButtons() {
  if (manualSelectedZone >= zonesCount) manualSelectedZone = 0;

  if (manualSelectPin >= 0 && manualSelectPin <= 39) {
    pinMode(manualSelectPin, INPUT_PULLUP);
  } else {
    manualSelectPin = -1;
  }

  if (manualStartPin >= 0 && manualStartPin <= 39) {
    pinMode(manualStartPin, INPUT_PULLUP);
  } else {
    manualStartPin = -1;
  }
}

void tickManualButtons() {
  const uint32_t nowMs = millis();
  if (zonesCount == 0) return;
  manualSelectedZone = manualSelectedZone % zonesCount;

  // Select button: cycle target zone
  static int lastSelState = HIGH;
  static uint32_t lastSelChange = 0;
  if (manualSelectPin >= 0) {
    int s = digitalRead(manualSelectPin);
    if (s != lastSelState && (nowMs - lastSelChange) > MANUAL_BTN_DEBOUNCE_MS) {
      lastSelChange = nowMs;
      lastSelState = s;
      if (s == LOW) {
        manualSelectedZone = (manualSelectedZone + 1) % zonesCount;
        Serial.printf("[BTN] Manual select -> Z%d\n", manualSelectedZone + 1);
        showManualSelection();
      }
    }
  }

  // Start/stop button: toggle the selected zone
  static int lastStartState = HIGH;
  static uint32_t lastStartChange = 0;
  if (manualStartPin >= 0) {
    int s = digitalRead(manualStartPin);
    if (s != lastStartState && (nowMs - lastStartChange) > MANUAL_BTN_DEBOUNCE_MS) {
      lastStartChange = nowMs;
      lastStartState = s;
      if (s == LOW) {
        uint8_t z = manualSelectedZone % zonesCount;
        if (zoneActive[z]) {
          turnOffValveManual(z);
        } else {
          turnOnValveManual(z);
        }
      }
    }
  }
}

// ---------- Weather / Forecast ----------
String fetchWeather() {
  if (apiKey.length()<5 || city.length()<1) return "";
  HTTPClient http; 
  http.setTimeout(2500); // NEW: shorter timeout
  String url="http://api.openweathermap.org/data/2.5/weather?id="+city+"&appid="+apiKey+"&units=metric";
  http.begin(client,url);
  int code=http.GET();
  if (code != 200) {
    http.end();
    return "";
  }
  String payload = http.getString();
  http.end();
  return payload;
}

String fetchForecast(float lat, float lon) {
  if (apiKey.length() < 5) return "";
  HTTPClient http; 
  http.setTimeout(3000); // NEW: shorter timeout

  String url = "http://api.openweathermap.org/data/2.5/onecall?lat=" + String(lat,6) +
               "&lon=" + String(lon,6) +
               "&appid=" + apiKey +
               "&units=metric&exclude=minutely,alerts";

  http.begin(client, url);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return "";
  }
  String payload = http.getString();
  http.end();
  return payload;
}

// ---------- NEW helpers for rain history ----------
// Rolling 24h rain history (ACTUAL ONLY, no forecast)
static void tickActualRainHistory() {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_r(&now, &lt);

  // Only record once at the start of each hour,
  // and make sure we don't double-count within 5 minutes.
  if (lt.tm_min == 0 && lt.tm_sec < 5 && (now - lastRainHistHour) > 300) {
    float v = rain1hNow;

    // Sanity: NaN / negative => 0
    if (!isfinite(v) || v < 0.0f) {
      v = 0.0f;
    }

    // Optional: clamp crazy hourly spikes from OWM
    const float MAX_HOURLY_MM = 20.0f;  // tweak to taste: 10, 15, 20, etc.
    if (v > MAX_HOURLY_MM) {
      v = MAX_HOURLY_MM;
    }

    // Advance ring buffer and store
    rainIdx = (rainIdx + 1) % 24;
    rainHist[rainIdx] = v;
    lastRainHistHour = now;

    // Optional debug:
    // Serial.printf("[RainHist] %02d:00 -> v=%.2f, sum24=%.2f\n",
    //               lt.tm_hour, v, last24hActualRain());
  }
}

// Sum rolling 24h actual rainfall from the ring buffer
static float last24hActualRain() {
  float total = 0.0f;
  for (int i = 0; i < 24; ++i) {
    float v = rainHist[i];
    if (!isfinite(v) || v < 0.0f) {
      v = 0.0f;
    }
    total += v;
  }
  return total;
}



void updateCachedWeather() {
  // NEW: never start fetches while handling HTTP requests
  if (g_inHttp) return;

  unsigned long nowms = millis();
  bool needCur = (cachedWeatherData == "" || (nowms - lastWeatherUpdate >= weatherUpdateInterval));
  bool haveCoord = false; float lat = NAN, lon = NAN;

  if (needCur) {
    String fresh = fetchWeather();
    if (fresh.length() > 0) {
      cachedWeatherData = fresh;
      lastWeatherUpdate = nowms;
    }
  }

  // Extract coordinates and 1h rain
  {
    DynamicJsonDocument js(1024);
    if (deserializeJson(js, cachedWeatherData) == DeserializationError::Ok) {
      if (js["coord"]["lat"].is<float>() && js["coord"]["lon"].is<float>()) {
        lat = js["coord"]["lat"].as<float>();
        lon = js["coord"]["lon"].as<float>();
        haveCoord = true;
      }

      // 1h only (no 3h fallback)
      float r1 = 0.0f;
      JsonVariant rv = js["rain"];
      if (!rv.isNull()) {
        if (rv.is<float>() || rv.is<double>() || rv.is<int>()) {
          r1 = rv.as<float>(); // rare form: rain: <number>
        } else if (rv["1h"].is<float>() || rv["1h"].is<double>() || rv["1h"].is<int>()) {
          r1 = rv["1h"].as<float>(); // common form: rain: { "1h": x }
        }
      }
      rain1hNow = r1;
    } else {
      rain1hNow = 0.0f;
    }
  }

  // ---- Forecast fetch / parse ----
  if (haveCoord && (cachedForecastData == "" || (nowms - lastForecastUpdate >= forecastUpdateInterval))) {
    String freshFc = fetchForecast(lat, lon);
    if (freshFc.length() > 0) {
      cachedForecastData = freshFc;
      lastForecastUpdate = nowms;

      rainNext12h_mm = 0; 
      rainNext24h_mm = 0; 
      popNext12h_pct = 0; 
      nextRainIn_h   = -1;
      maxGust24h_ms  = 0; 
      todaySunrise   = 0;   
      todaySunset    = 0;

      DynamicJsonDocument fc(14 * 1024);
      if (deserializeJson(fc, cachedForecastData) == DeserializationError::Ok) {
      if (fc["daily"].is<JsonArray>() && fc["daily"].size() > 0) {
        JsonObject d0 = fc["daily"][0];
        todaySunrise = (time_t)(d0["sunrise"] | 0);
        todaySunset  = (time_t)(d0["sunset"]  | 0);
      }
      auto read1h = [](JsonVariant v) -> float {
        if (v.isNull()) return 0.0f;
        if (v.is<float>() || v.is<double>() || v.is<int>()) return v.as<float>();
        JsonVariant one = v["1h"];
        return one.isNull() ? 0.0f : one.as<float>();
      };
      if (fc["hourly"].is<JsonArray>()) {
        JsonArray hr = fc["hourly"].as<JsonArray>();
        int hrs = hr.size();
        int L24 = min(24, hrs);
        int L12 = min(12, hrs);
        for (int i = 0; i < L24; i++) {
          JsonObject h = hr[i];
          float r = 0.0f;
          r += read1h(h["rain"]);
          r += read1h(h["snow"]);
          if (i < L12) {
            rainNext12h_mm += r;
            int pop = (int)roundf(100.0f * (h["pop"] | 0.0f));
            if (pop > popNext12h_pct) popNext12h_pct = pop;
          }
            rainNext24h_mm += r;
            if (nextRainIn_h < 0) {
              float popf = h["pop"] | 0.0f;
              if (r > 0.01f || popf >= 0.5f) nextRainIn_h = i;
            }
          float g = h["wind_gust"] | 0.0f;
          if (g > maxGust24h_ms) maxGust24h_ms = g;
        }
      }
        if (rainNext24h_mm <= 0.0f && fc["daily"].is<JsonArray>() && fc["daily"].size() > 0) {
          JsonObject d0 = fc["daily"][0];
          float dailyTotal = 0.0f;
          if (!d0["rain"].isNull()) {
            if (d0["rain"].is<float>() || d0["rain"].is<double>() || d0["rain"].is<int>())
              dailyTotal += d0["rain"].as<float>();
          }
          if (!d0["snow"].isNull()) {
            if (d0["snow"].is<float>() || d0["snow"].is<double>() || d0["snow"].is<int>())
              dailyTotal += d0["snow"].as<float>();
          }
          if (dailyTotal > 0.0f) {
            rainNext24h_mm = dailyTotal;
            if (rainNext12h_mm <= 0.0f) rainNext12h_mm = dailyTotal * 0.5f;
          }
        }
      }
      if (isnan(rainNext12h_mm) || rainNext12h_mm < 0) rainNext12h_mm = 0.0f;
      if (isnan(rainNext24h_mm) || rainNext24h_mm < 0) rainNext24h_mm = 0.0f;
      if (isnan(maxGust24h_ms)  || maxGust24h_ms  < 0) maxGust24h_ms  = 0.0f;
    }
  }

  // Fallback sunrise/sunset & min/max from current weather
  DynamicJsonDocument cur(2048);
  if (deserializeJson(cur, cachedWeatherData) == DeserializationError::Ok) {
    time_t sr = (time_t)(cur["sys"]["sunrise"] | 0);
    time_t ss = (time_t)(cur["sys"]["sunset"]  | 0);
    if (sr > 0) todaySunrise = sr;
    if (ss > 0) todaySunset  = ss;
    float tcur = cur["main"]["temp"] | NAN;
    if (isfinite(tcur)) {
      if (!isfinite(todayMin_C) || tcur < todayMin_C) todayMin_C = tcur;
      if (!isfinite(todayMax_C) || tcur > todayMax_C) todayMax_C = tcur;
    }
  }

  // Roll hourly history
  tickActualRainHistory();
}

// NEW: called only from loop()
void tickWeather(){
  static uint32_t lastTick = 0;
  uint32_t nowMs = millis();
  const uint32_t WEATHER_TICK_MS = 10000UL; // every 10s is plenty

  if (nowMs - lastTick < WEATHER_TICK_MS) {
    return;
  }
  lastTick = nowMs;

  updateCachedWeather();
}

// ---------- Rain/Wind logic with cooldown & threshold ----------
bool checkWindRain() {
  // Throttle to at most once per second
  static uint32_t lastCheckMs = 0;
  static bool     lastResult  = false;
  // Track only the kind of "rain" that is allowed to start cooldown
  static bool     prevCoolRain = false;

  uint32_t nowMs = millis();
  if (nowMs - lastCheckMs < 1000UL) {
    // Reuse last computed state (rainActive/windActive already set)
    return lastResult;
  }
  lastCheckMs = nowMs;

  bool newWeatherRainActual = false;   // raw "is it raining now" from OWM
  bool newSensorRainActual  = false;   // raw sensor state
  bool newWindActual        = false;   // raw wind above threshold

  // --- 1) Parse cached weather JSON (OpenWeather) ---
  DynamicJsonDocument js(1024);
  DeserializationError err = deserializeJson(js, cachedWeatherData);

  if (!err) {
    // ----- RAIN BY WEATHER (instant, use 1h / condition code) -----
    float rain1hRaw = js["rain"]["1h"] | 0.0f;

    float rain1hLogic = rain1hRaw;
    if (rain1hLogic < 0.0f) rain1hLogic = 0.0f;
    const float MAX_RAIN1H_FOR_LOGIC = 20.0f;  // mm, tweak if needed
    if (rain1hLogic > MAX_RAIN1H_FOR_LOGIC) {
      rain1hLogic = MAX_RAIN1H_FOR_LOGIC;
    }

    if (rain1hLogic > 0.0f) {
      newWeatherRainActual = true;
    } else {
      // Fallback: weather condition code (2xx/3xx/5xx = rain-ish)
      int wid = js["weather"][0]["id"] | 0;
      if (wid >= 200 && wid < 600) {
        newWeatherRainActual = true;
      }
    }

    // ----- WIND DELAY (raw) -----
    float windSpeed = js["wind"]["speed"] | 0.0f;  // m/s
    if (windSpeedThreshold > 0.0f) {
      newWindActual = (windSpeed >= windSpeedThreshold);
    } else {
      newWindActual = false;
    }
  } else {
    newWeatherRainActual = false;
    newWindActual        = false;
  }

  // --- 2) Physical rain sensor (raw state) ---
  newSensorRainActual = physicalRainNowRaw();

  // --- 3) Accumulated rainfall (24h ACTUAL ONLY) ---
  float actual24 = last24hActualRain();
  bool aboveThreshold = false;
  if (rainThreshold24h_mm > 0) {
    if (actual24 >= rainThreshold24h_mm) {
      aboveThreshold = true;
    }
  }

  // --- 4) Apply feature gates and build "effective" flags ---
  bool effectiveInstantRain =
      rainDelayEnabled &&
      newWeatherRainActual;

  // 24h threshold rain: only when actual 24h rainfall >= user threshold.
  bool effectiveThresholdRain =
      rainDelayEnabled &&
      (rainThreshold24h_mm > 0) &&
      aboveThreshold;

  // Physical sensor rain: used for both blocking + cooldown.
  bool effectiveSensorRain =
      rainDelayEnabled &&
      rainSensorEnabled &&
      newSensorRainActual;

  // Public flags / state
  rainByWeatherActive = effectiveInstantRain || effectiveThresholdRain;
  rainBySensorActive  = effectiveSensorRain;
  rainActive          = (rainByWeatherActive || rainBySensorActive);

  windActive = (windDelayEnabled && newWindActual);

  // --- 5) Cooldown logic (ONLY threshold/sensor-based rain) ---
  // Define which rain sources are allowed to start cooldown when they clear:
  //  - 24h threshold actual rainfall
  //  - Physical rain sensor
  bool coolRainNow = (effectiveThresholdRain || effectiveSensorRain);

  time_t now = time(nullptr);

  // (a) If we *previously* had threshold/sensor rain, and now it's clear,
  //     start cooldown window.
  if (prevCoolRain && !coolRainNow) {
    if (rainDelayEnabled && rainCooldownHours > 0) {
      rainCooldownUntilEpoch =
          (uint32_t)now + (uint32_t)rainCooldownHours * 3600UL;
    } else {
      rainCooldownUntilEpoch = 0;
    }
  }

  // (b) While threshold/sensor rain is active, there is no cooldown.
  if (coolRainNow) {
    rainCooldownUntilEpoch = 0;
  }

  // (c) When cooldown expires, clear it.
  if (!coolRainNow &&
      rainCooldownUntilEpoch > 0 &&
      (uint32_t)now >= rainCooldownUntilEpoch) {
    rainCooldownUntilEpoch = 0;
  }

  // Track last 24h amount for UI (RainScreen "Last: x.xx mm")
  lastRainAmount = actual24;

  // Remember for next edge detection
  prevCoolRain = coolRainNow;

  // Final combined state that the rest of the code cares about
  lastResult = (rainActive || windActive);
  return lastResult;
}

// ---------- Event log ----------
void logEvent(int zone, const char* eventType, const char* source, bool rainDelayed) {
  updateCachedWeather(); // safe early-out if g_inHttp==true, keeps details recent enough
  DynamicJsonDocument js(512);
  float temp=NAN, wind=NAN; int hum=0; String cond="?", cname="-";
  if (deserializeJson(js,cachedWeatherData)==DeserializationError::Ok) {
    temp = js["main"]["temp"].as<float>();
    hum  = js["main"]["humidity"].as<int>();
    wind = js["wind"]["speed"].as<float>();
    cond = js["weather"][0]["main"].as<const char*>();
    cname= js["name"].as<const char*>();
  }

  File f = LittleFS.open("/events.csv","a");
  if (!f) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char ts[32];
  sprintf(ts,"%04d-%02d-%02d %02d:%02d:%02d", t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

  String line; line.reserve(200);
  line += ts; line += ","; line += (zone>=0?zoneNames[zone]:String("n/a")); line += ",";
  line += eventType; line += ","; line += source; line += ",";
  line += (rainDelayed?"Active":"Off"); line += ",";
  line += String(temp,1); line += ","; line += String(hum); line += ",";
  line += String(wind,1); line += ","; line += cond; line += ","; line += cname; line += "\n";

  f.print(line); f.close();
}

// ---------- OLED UI ----------
void toggleBacklight(){ static bool inverted=false; inverted=!inverted; display.invertDisplay(inverted); }

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate=0; unsigned long now=millis();
  if (now - lastUpdate < 1000) return; lastUpdate = now;

  unsigned long elapsed=(now - zoneStartMs[zone]) / 1000;
  unsigned long total=zoneRunTotalSec[zone];
  if (total==0) total = durationForSlot(zone,1);
  unsigned long rem = (elapsed<total ? total - elapsed : 0);

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0); display.print(zoneNames[zone]); display.print(" ");
  display.print(elapsed/60); display.print(":"); if ((elapsed%60)<10) display.print('0'); display.print(elapsed%60);

  display.setCursor(0,12);
  if (elapsed<total){ display.print(rem/60); display.print("m Remaining"); }
  else display.print("Complete");
  display.display();
}

void RainScreen(){
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(0,0); display.print(isPausedNow()? "System Pause" : "Rain/Wind");
  display.setTextSize(1);
  display.setCursor(0,20); display.printf("Last: %.2f mm", lastRainAmount);
  display.setCursor(0,32); display.print("Cause: "); display.print(rainDelayCauseText());
  int delayed=0; for (int i=0;i<(int)zonesCount;i++) if (pendingStart[i]) delayed++;
  display.setCursor(0,46); display.printf("Queued: %d", delayed);
  display.display();
}

void HomeScreen() {
  DynamicJsonDocument js(1024); (void)deserializeJson(js,cachedWeatherData);

  float temp = js["main"]["temp"].as<float>();
  int   hum  = js["main"]["humidity"].as<int>();
  int   pct  = tankPercent();

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0,0); display.printf("%02d:%02d", t->tm_hour, t->tm_min);
  display.setTextSize(1);
  display.setCursor(80,3); display.printf("%02d/%02d", t->tm_mday, t->tm_mon+1);
  display.setCursor(66,3); display.print( (t->tm_isdst>0) ? "DST" : "STD" );

  display.setCursor(0,20);
  if (!isnan(temp) && hum >= 0) {
    display.printf("T:%2.0fC H:%02d%%", temp, hum);
  } else {
    display.print("T:-- H:--");
  }

  display.setCursor(0,32);
  if (tankEnabled) { display.printf("Tank:%3d%% ", pct); display.print(isTankLow()? "(Mains)":"(Tank)"); }
  else { display.print("Zones: "); display.print(zonesCount); }

  for (int i=0;i<i_min(2,(int)zonesCount);i++){
    int x=(i==0)?0:64;
    display.setCursor(x,44);
  if (zoneActive[i]) {
    unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
    unsigned long total=zoneRunTotalSec[i];
    if (total==0) total = durationForSlot(i,1);
    unsigned long rem=(elapsed<total?total-elapsed:0);
    display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
  } else display.printf("Z%d:Off", i+1);
}
for (int i=2;i<i_min(4,(int)zonesCount);i++){
  int x=(i==2)?0:64;
  display.setCursor(x,56);
  if (zoneActive[i]) {
    unsigned long elapsed=(millis()-zoneStartMs[i])/1000;
    unsigned long total=zoneRunTotalSec[i];
    if (total==0) total = durationForSlot(i,1);
    unsigned long rem=(elapsed<total?total-elapsed:0);
    display.printf("Z%d:%02d:%02d", i+1, (int)(rem/60),(int)(rem%60));
  } else display.printf("Z%d:Off", i+1);
}
  display.display();
}

bool shouldStartZone(int zone) {
  if (zone < 0 || zone >= (int)MAX_ZONES) return false;

  time_t now = time(nullptr);
  struct tm* tt = localtime(&now);
  if (!tt) return false;

  const int wd = tt->tm_wday; // Sun=0..Sat=6
  const int hr = tt->tm_hour;
  const int mn = tt->tm_min;

  if (lastCheckedMinute[zone] == mn) return false;      // avoid dup triggers
  if (!days[zone][wd]) return false;                    // day not enabled
  // zero duration guard
  if (durationMin[zone] == 0 && durationSec[zone] == 0 && duration2Min[zone]==0 && duration2Sec[zone]==0) return false;

  const bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  const bool match2 = (enableStartTime2[zone] && hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) { 
    lastCheckedMinute[zone] = mn; 
    lastStartSlot[zone] = match2 ? 2 : 1;
    if (durationForSlot(zone,lastStartSlot[zone]) == 0) return false;
    return true; 
  }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed=(millis()-zoneStartMs[zone])/1000;
  unsigned long total=zoneRunTotalSec[zone];
  if (total==0) total = durationForSlot(zone,1);
  return (elapsed >= total);
}

void turnOnZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;

  checkWindRain();
  Serial.printf("[VALVE] Request ON Z%d rain=%d wind=%d blocked=%d\n",
                z+1, rainActive?1:0, windActive?1:0, isBlockedNow()?1:0);

  if (isBlockedNow())  { cancelStart(z, "BLOCKED", false); return; }
  if (rainActive)      { cancelStart(z, "RAIN",    true ); return; }
  if (windActive)      { cancelStart(z, "WIND",    false); return; }

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (pin < 0 || pin > 39) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping\n", z+1);
      cancelStart(z, "NO_PIN", false);
      return;
    }
  }

  bool anyOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyOn = true; break; }
  }
  if (!runZonesConcurrent && anyOn) {
    pendingStart[z] = true;
    logEvent(z, "QUEUED", "ACTIVE RUN", false);
    return;
  }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  unsigned long total = durationForSlot(z, lastStartSlot[z]);
  if (total == 0) total = durationForSlot(z, 1);
  zoneRunTotalSec[z] = total;

  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  if (!usePcf) {
    gpioZoneWrite(z, true);   // ON via GPIO
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    // PCF8574 path: active-LOW
    pcfOut.digitalWrite(PCH[z], LOW);
    setWaterSourceRelays(mainsOn, tankOn);
  }

  logEvent(z, "START", src, false);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(2, 0);
  display.print(zoneNames[z]);
  display.print(" ON");
  display.display();
  delay(350);
  HomeScreen();
}

void turnOffZone(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  Serial.printf("[VALVE] Request OFF Z%d\n", z+1);
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  bool wasDelayed = rainActive || windActive || isPausedNow() ||
                    !systemMasterEnabled ||
                    (rainCooldownUntilEpoch > time(nullptr));
  logEvent(z, "STOPPED", src, wasDelayed);

  const bool usePcf = useExpanderForZone(z);

  if (!usePcf) {
    // OFF the requested zone
    gpioZoneWrite(z, false);
  } else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) { anyStillOn = true; break; }
  }
  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(4, 0);
  display.print(zoneNames[z]);
  display.print(" OFF");
  display.display();
  delay(300);
}

void turnOnValveManual(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  if (zoneActive[z])   return;

  const bool usePcf = useExpanderForZone(z);
  if (!usePcf) {
    int pin = zonePins[z];
    if (pin < 0 || pin > 39) {
      Serial.printf("[VALVE] Z%d has no GPIO pin assigned; skipping manual start\n", z+1);
      cancelStart(z, "NO_PIN", false);
      return;
    }
  }

  // Respect run mode overlap rules
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      if (!runZonesConcurrent) {
        Serial.println("[VALVE] Manual start blocked: another zone is running");
        return;
      }
      break; // concurrent: allow additional zone
    }
  }

  // cancel manual requests during block/delay instead of queueing
  if (isBlockedNow())  { cancelStart(z, "BLOCKED", false); return; }
  if (rainActive)      { cancelStart(z, "RAIN",    true ); return; }
  if (windActive)      { cancelStart(z, "WIND",    false); return; }

  zoneStartMs[z] = millis();
  zoneActive[z] = true;
  lastStartSlot[z] = 1;
  zoneRunTotalSec[z] = durationForSlot(z,1);
  const char* src = "None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  if (!usePcf) {
    gpioZoneWrite(z, true);   // ON via GPIO
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], LOW);
    setWaterSourceRelays(mainsOn, tankOn);
  }

  logEvent(z, "MANUAL START", src, false);
}

void turnOffValveManual(int z) {
  if (z < 0 || z >= (int)zonesCount || z >= (int)MAX_ZONES) return;
  if (!zoneActive[z]) return;

  const bool usePcf = useExpanderForZone(z);
  const char* src="None";
  bool mainsOn=false, tankOn=false;
  chooseWaterSource(src, mainsOn, tankOn);

  // Turn this zone OFF
  if (!usePcf) {
    // Use configured polarity for GPIO fallback
    gpioZoneWrite(z, false);          // OFF
  } else {
    // PCF8574 path: keep active-LOW semantics (HIGH = OFF)
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;
  zoneRunTotalSec[z] = 0;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      anyStillOn = true;
      break;
    }
  }

  if (anyStillOn) {
    setWaterSourceRelays(mainsOn, tankOn);
  } else {
    setWaterSourceRelays(false, false);   // mains OFF, tank OFF
  }
}

// ---------- Next Water (queue-first) ----------
static NextWaterInfo computeNextWatering() {
  NextWaterInfo best{0, -1, 0};

  for (int z=0; z<(int)zonesCount; ++z) {
    if (pendingStart[z]) {
      best.epoch = time(nullptr);
      best.zone  = z;
      best.durSec = (uint32_t)durationForSlot(z, lastStartSlot[z]);
      return best;
    }
  }

  time_t now = time(nullptr);
  struct tm base; localtime_r(&now, &base);

  auto consider = [&](int z, int hr, int mn, bool enabled, int slot) {
    if (!enabled) return;
    unsigned long tot = durationForSlot(z, slot);
    if (tot == 0) return;

    struct tm cand = base;
    cand.tm_sec  = 0;
    cand.tm_hour = hr;
    cand.tm_min  = mn;

    time_t t = mktime(&cand);
    if (t <= now) t += 24*60*60;

    for (int k = 0; k < 8; ++k) {
      struct tm tmp; localtime_r(&t, &tmp);
      int wd = tmp.tm_wday;
      if (days[z][wd]) {
        if (best.zone < 0 || t < best.epoch) {
          best.epoch = t;
          best.zone  = z;
          best.durSec = (uint32_t)tot;
        }
        return;
      }
      t += 24*60*60;
    }
  };

  for (int z=0; z<(int)zonesCount; ++z) {
    consider(z, startHour[z],  startMin[z],  true, 1);
    consider(z, startHour2[z], startMin2[z], enableStartTime2[z], 2);
  }
  return best;
}

// Main Page
void handleRoot() {
  HttpScope _scope;  // NEW: mark that we're in an HTTP handler so no blocking fetches

  // --- Precompute state / snapshots ---
  checkWindRain();

  time_t now = time(nullptr);
  struct tm* ti = localtime(&now);
  char timeStr[9], dateStr[11];
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", ti);
  strftime(dateStr, sizeof(dateStr), "%d/%m/%Y", ti);

  // Keep this - it respects the g_inHttp guard
  updateCachedWeather();
  DynamicJsonDocument js(1024);
  DeserializationError werr = deserializeJson(js, cachedWeatherData);

  // Safe reads
  float temp = NAN, hum = NAN, ws = NAN, feels = NAN;
  if (!werr) {
    if (js["main"]["temp"].is<float>())        temp  = js["main"]["temp"].as<float>();
    if (js["main"]["feels_like"].is<float>())  feels = js["main"]["feels_like"].as<float>();
    if (js["main"]["humidity"].is<float>())    hum   = js["main"]["humidity"].as<float>();
    if (js["wind"]["speed"].is<float>())       ws    = js["wind"]["speed"].as<float>();
  }

  String cond = werr ? "-" : String(js["weather"][0]["main"].as<const char*>());
  if (cond == "") cond = "-";
  String cityName = werr ? "-" : String(js["name"].as<const char*>());
  if (cityName == "") cityName = "-";

  const int    tankPct   = tankPercent();
  const String causeText = rainDelayCauseText();

  // --- HTML ---
  String html; html.reserve(40000);
  html += F("<!doctype html><html lang='en' data-theme='light'><head>");
  html += F("<meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>ESP32 Irrigation</title>");
  html += F("<style>");
  html += F(".center{max-width:1280px;margin:0 auto}");
  html += F(":root[data-theme='light']{--bg:#ecf1f8;--bg2:#f6f8fc;--glass:rgba(255,255,255,.55);--glass-brd:rgba(140,158,190,.35);--panel:#fff;--line:#d5dfef;");
  html += F("--card:#ffffff;--ink:#0f172a;--muted:#667084;--primary:#1c74d9;--primary-2:#1160b6;--ok:#16a34a;--warn:#d97706;--bad:#dc2626;");
  html += F("--chip:#eef4ff;--chip-brd:#cfe1ff;--ring:#dfe8fb;--ring2:#a4c6ff;--shadow:0 18px 40px rgba(19,33,68,.15)}");
  html += F(":root[data-theme='dark']{--bg:#0a0f18;--bg2:#0e1624;--glass:rgba(16,26,39,.6);--glass-brd:rgba(96,120,155,.28);--panel:#101826;--line:#223a5e;");
  html += F("--card:#101826;--ink:#e8eef6;--muted:#9fb0ca;--primary:#52a7ff;--primary-2:#2f7fe0;--ok:#22c55e;--warn:#f59e0b;--bad:#ef4444;");
  html += F("--chip:#0f2037;--chip-brd:#223a5e;--ring:#172a46;--ring2:#2c4f87;--shadow:0 18px 40px rgba(0,0,0,.45)}");
  html += F("*{box-sizing:border-box}html,body{margin:0;padding:0;");
  html += F("background:radial-gradient(1200px 600px at 10% -5%,var(--bg2),transparent),");
  html += F("radial-gradient(1200px 700px at 100% 0%,var(--ring),transparent),");
  html += F("radial-gradient(900px 500px at -10% 80%,var(--ring2),transparent),var(--bg);");
  html += F("color:var(--ink);font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif}");
  html += F(":root{--gap:16px;--pad:18px;--pad-lg:22px;--radius:20px;--radius-sm:16px;}");

  // Top nav - made a bit tighter on mobiles
  html += F(".nav{position:sticky;top:0;z-index:10;padding:10px 12px 12px;");
  html += F("background:linear-gradient(180deg,rgba(0,0,0,.25),transparent),var(--primary-2);");
  html += F("box-shadow:0 16px 36px rgba(0,0,0,.25)}");
  html += F(".nav .in{max-width:1280px;margin:0 auto;display:flex;align-items:center;justify-content:space-between;gap:12px;color:#fff;flex-wrap:wrap}");
  html += F(".brand{display:flex;align-items:center;gap:8px;font-weight:800;letter-spacing:.2px;font-size:1.12rem}");
  html += F(".dot{width:12px;height:12px;border-radius:999px;background:#84ffb5;box-shadow:0 0 14px #84ffb5}");
  html += F(".nav .meta{display:flex;gap:8px;flex-wrap:wrap;align-items:center;font-weight:650;font-size:.88rem}");
  html += F(".pill{background:rgba(255,255,255,.16);border:1px solid rgba(255,255,255,.28);border-radius:999px;padding:7px 12px}");
  html += F(".btn-ghost{background:rgba(255,255,255,.14);border:1px solid rgba(255,255,255,.35);color:#fff;");
  html += F("border-radius:10px;padding:8px 14px;font-weight:700;cursor:pointer;font-size:.92rem}");

  // Cards and grids
  html += F(".wrap{max-width:1280px;margin:20px auto;padding:0 16px}");
  html += F(".glass{background:var(--glass);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow)}");
  html += F(".section{padding:var(--pad)}");
  html += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:var(--gap)}");
  html += F(".card{background:var(--card);border:1px solid var(--glass-brd);border-radius:var(--radius);box-shadow:var(--shadow);padding:var(--pad)}");
  html += F(".card h3{margin:0 0 10px 0;font-size:1.15rem;color:var(--ink);font-weight:850;letter-spacing:.35px;");
  html += F("padding-bottom:6px;border-bottom:2px solid var(--primary);display:flex;align-items:center;gap:8px}");
  html += F(".card h4{margin:6px 0 6px 0;font-size:.95rem;color:var(--muted);font-weight:700;letter-spacing:.2px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:8px 14px;font-weight:650;white-space:nowrap;font-size:.95rem;color:var(--ink)}");
  html += F(".big{font-weight:800;font-size:1.3rem}.hint{color:var(--muted);font-size:.9rem;margin-top:4px}.sub{color:var(--muted);font-size:.88rem}");
  html += F(".meter{position:relative;height:18px;border-radius:999px;background:linear-gradient(180deg,rgba(0,0,0,.12),transparent);border:1px solid var(--glass-brd);overflow:hidden;margin-top:6px}");
  html += F(".fill{position:absolute;inset:0 0 0 0;width:0%;height:100%;background:linear-gradient(90deg,#30d1ff,#4da3ff,#1c74d9);");
  html += F("box-shadow:0 0 30px rgba(77,163,255,.35) inset;transition:width .4s ease}");
  html += F(".badge{display:inline-flex;align-items:center;gap:6px;padding:8px 13px;border-radius:999px;border:1px solid var(--glass-brd);font-size:.9rem}");
  html += F(".b-ok{background:rgba(34,197,94,.12);border-color:rgba(34,197,94,.35)}");
  html += F(".b-warn{background:rgba(245,158,11,.12);border-color:rgba(245,158,11,.35)}");
  html += F(".b-bad{background:rgba(239,68,68,.12);border-color:rgba(239,68,68,.38)}");
  html += F(".toolbar{display:flex;gap:var(--gap);flex-wrap:wrap;margin:14px 0 0}");
  html += F(".btn{background:linear-gradient(180deg,var(--primary),var(--primary-2));color:#fff;border:1px solid rgba(0,0,0,.08);border-radius:13px;padding:11px 16px;font-weight:800;cursor:pointer;");
  html += F("box-shadow:0 8px 20px rgba(0,0,0,.22);font-size:1.02rem}");
  html += F(".btn-secondary{background:transparent;color:var(--ink);border:1px solid var(--line);box-shadow:none}");
  html += F(".btn:disabled{background:#7f8aa1;cursor:not-allowed;box-shadow:none}.btn-danger{background:linear-gradient(180deg,#ef4444,#b91c1c);border-color:rgba(185,28,28,.6)}");
  html += F(".btn,.btn-ghost,.pill{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active:not(:disabled),.btn-ghost:active,.pill:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-ghost,.pill{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".zone-actions{margin-top:8px;justify-content:flex-end;align-items:center}");
  html += F(".zones{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:var(--gap);justify-content:center;justify-items:stretch}");
  html += F(".zone-card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius-sm);padding:var(--pad);");
  html += F("display:flex;flex-direction:column;gap:12px;min-height:140px}");
  html += F(".zone-head{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:2px}");
  html += F(".zone-title{display:flex;align-items:center;gap:8px;font-weight:700;font-size:1.08rem;min-width:0}");
  html += F(".zone-index{width:26px;height:26px;border-radius:999px;display:flex;align-items:center;justify-content:center;");
  html += F("font-size:.82rem;background:var(--chip);border:1px solid var(--chip-brd);flex-shrink:0}");
  html += F(".zone-dot{width:13px;height:13px;border-radius:999px;background:var(--line);border:1px solid var(--glass-brd);box-shadow:0 0 0 2px rgba(0,0,0,.06) inset}");
  html += F(".zone-dot.on{background:var(--ok);box-shadow:0 0 10px rgba(34,197,94,.55)}");
  html += F(".zone-title .big{font-size:.98rem;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}");
  html += F(".zone-timer{display:flex;align-items:center;gap:10px;margin-top:4px}");
  html += F(".zone-rem-wrap{min-width:90px}");
  html += F(".zone-rem-label{font-size:.8rem;letter-spacing:.4px;text-transform:uppercase;color:var(--muted);display:block;margin-bottom:2px}");
  html += F(".zone-rem{font-size:1.05rem;font-weight:700;color:var(--ink)}");
  html += F(".zone-meter{flex:1;display:flex;flex-direction:column;gap:3px}");
  html += F(".zone-meter .meter{margin-top:0}");
  html += F(".zone-bar-label{font-size:.82rem;letter-spacing:.3px;color:var(--muted);font-weight:600}");
  html += F(".zone-meta-row{display:flex;gap:6px;flex-wrap:wrap;margin-top:6px;font-size:.82rem;color:var(--muted)}");
  html += F(".zone-meta-row .pill-soft{padding:4px 10px;border-radius:999px;background:var(--chip);border:1px solid var(--chip-brd)}");
  html += F(".zone-meta-row b{font-weight:700}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 16px 34px rgba(0,0,0,.18)}");
  html += F(".btn:hover{filter:brightness(1.05)}");
  html += F(".chip:hover,.pill:hover{filter:brightness(1.02)}");
  html += F(".btn:focus-visible,.pill:focus-visible,.chip:focus-visible{outline:2px solid var(--primary);outline-offset:2px}");

  // Schedules styles (collapsible, mobile-friendly)
  html += F(".sched{margin-top:var(--gap)}");
  html += F(".sched-ctr{--schedW:360px;--gap:var(--gap);max-width:100%;margin:0 auto}");
  html += F(".sched-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:var(--gap)}");
  html += F(".sched-card{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius-sm);padding:var(--pad)}");
  html += F(".sched-card h4{margin:0 0 10px 0;font-size:1.05rem;font-weight:800;color:var(--ink);display:flex;align-items:center;gap:8px}");
  html += F(".sched-badge{background:var(--chip);border:1px solid var(--chip-brd);border-radius:999px;padding:4px 9px;font-size:.8rem;font-weight:700;color:var(--muted)}");
  html += F(".rowx{display:grid;grid-template-columns:110px 1fr;gap:10px;align-items:center;margin:10px 0}");
  html += F(".rowx label{min-width:0;font-size:.82rem;color:var(--muted);font-weight:700;letter-spacing:.4px;text-transform:uppercase}");
  html += F(".field{display:flex;align-items:center;gap:8px;flex-wrap:wrap}");
  html += F(".field.inline .in{width:72px}");
  html += F(".field .sep{color:var(--muted);font-weight:700}");
  html += F(".field .unit{color:var(--muted);font-size:.85rem}");
  html += F(".toggle-inline{display:inline-flex;align-items:center;gap:6px;font-size:.85rem;color:var(--muted)}");
  html += F(".in{border:1px solid var(--line);border-radius:10px;padding:8px 10px;background:transparent;color:var(--ink);font-size:.9rem}");
  html += F(".days-grid{display:flex;flex-wrap:wrap;gap:8px;justify-content:center}");
  html += F(".day{display:inline-flex;gap:6px;align-items:center;border:1px solid var(--line);border-radius:999px;padding:7px 12px;font-size:.85rem;background:var(--chip);border-color:var(--chip-brd)}");
  html += F(".day input{margin:0}");
  html += F("@media(max-width:720px){.nav .in{flex-direction:column;align-items:flex-start}.zones{grid-template-columns:1fr}.sched-grid{grid-template-columns:1fr}.rowx{grid-template-columns:1fr}.rowx label{margin-bottom:4px}}");
  html += F(".collapse{cursor:pointer;user-select:none;display:flex;align-items:center;justify-content:space-between;font-size:1.05rem}");
  html += F(".collapse .arr{font-size:1rem;opacity:.8;margin-left:6px}");
  html += F(".sched-title{display:flex;flex-direction:column;gap:2px}");
  html += F(".sched-sub{font-size:.88rem;color:var(--muted);font-weight:600}");

  // Desktop enhancements
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{margin:20px auto;padding:0 20px;}");
  html += F(".glass.section{padding:var(--pad-lg);}");
  html += F(".summary-grid{grid-template-columns:repeat(4,minmax(0,1fr));}");
  html += F(".zones{grid-template-columns:repeat(3,minmax(300px,1fr));}");
  html += F(".card{padding:var(--pad-lg);}");
  html += F(".card h3{font-size:1.12rem;}");
  html += F("}");
  html += F("</style></head><body>");

  // --- Nav ---
  html += F("<div class='nav'><div class='in'>");
  html += F("<div class='brand'><span class='dot'></span><span>ESP32 Irrigation</span></div>");
  html += F("<div class='meta'>");
  html += F("<span class='pill' id='clock'>"); html += timeStr; html += F("</span>");
  html += F("<span class='pill'>"); html += dateStr; html += F("</span>");
  html += F("<span class='pill'>"); html += ((ti && ti->tm_isdst>0) ? "DST" : "STD"); html += F("</span>");
  html += F("<span class='pill' title='IP ");
  html += WiFi.localIP().toString();
  html += F("'>espirrigation.local</span>");
  html += F("<button id='btn-master' class='pill' style='cursor:pointer;border:none' aria-pressed='");
  html += (systemMasterEnabled ? "true" : "false");
  html += F("' title='Toggle master enable/disable'>Master: <b id='master-state'>");
  html += (systemMasterEnabled ? "On" : "Off");
  html += F("</b></button>");
  html += F("<button id='themeBtn' class='btn-ghost' title='Toggle theme'>Theme</button>");
  html += F("</div></div></div>");

  // --- Summary cards ---
  html += F("<div class='wrap'><div class='glass section'><div class='grid summary-grid'>");

  // NEW location card with OpenWeatherMap link
  html += F("<div class='card'><h3>Location</h3>"
            "<a class='chip' id='owmLink' href='#' target='_blank' rel='noopener'>"
            "<b id='cityName'>");
html += cityName;   // initial label; JS will overwrite from /status
html += F("</b></a></div>");



  html += F("<div class='card'><h3>Local Time</h3><div id='upChip' class='big'>--:--:--</div><div class='hint'>Device timezone</div></div>");

  html += F("<div class='card'><h3>Wifi Signal</h3><div id='rssiChip' class='big'>");
  html += String(WiFi.RSSI()); html += F(" dBm</div><div class='hint'>Wi-Fi RSSI</div></div>");

  html += F("<div class='card'><h3>Tank Level</h3><div class='zone-head'>");
  html += F("<div class='big'><span id='tankPctLabel'>"); html += String(tankPct); html += F("%</span></div>");
  html += F("<div id='srcChip' class='sub'>"); html += sourceModeText(); html += F("</div></div>");
  html += F("<div class='meter'><div id='tankFill' class='fill' style='width:"); html += String(tankPct); html += F("%'></div></div></div>");

  html += F("<div class='card'><h3>Current Weather</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px'>");
  html += F("<div class='chip'>Temp <span id='tempChip'>"); html += (isnan(temp) ? String("--") : String(temp,1)+" C"); html += F("</span> <span id='tempTrend' style='font-weight:900;'>→</span></div>");
  html += F("<div class='chip'>Feels <span id='feelsChip'>"); html += (isnan(feels) ? String("--") : String(feels,1)+" C"); html += F("</span></div>");
  html += F("<div class='chip'>Humidity <span id='humChip'>");  html += (isnan(hum)  ? String("--") : String((int)hum)+" %"); html += F("</span></div>");
  html += F("<div class='chip'>Wind <span id='windChip'>"); html += (isnan(ws)   ? String("--") : String(ws,1)+" m/s"); html += F("</span></div>");
  html += F("<div class='chip'>Condition <b id='cond'>");
  html += cond.length() ? cond : String("--");
  html += F("</b></div></div></div>");

  html += F("<div class='card'><h3>Weather Today</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:6px'>");
  html += F("<div class='chip'><b>Low</b>&nbsp;<b id='tmin'>---</b> C</div>");
  html += F("<div class='chip'><b>High</b>&nbsp;<b id='tmax'>---</b> C</div>");
  html += F("<div class='chip'><b>Pressure</b>&nbsp;<b id='press'>--</b> hPa</div>");
  html += F("<div class='chip'><span class='sub'>Sunrise</span>&nbsp;<b id='sunr'>--:--</b></div>");
  html += F("<div class='chip'><span class='sub'>Sunset</span>&nbsp;<b id='suns'>--:--</b></div>");
  html += F("</div></div>");

  // Delays / status
  html += F("<div class='card'><h3>Delays</h3><div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:6px'>");
  html += F("<div id='rainBadge' class='badge "); html += (rainActive ? "b-bad" : "b-ok"); html += F("'>Rain: <b>");
  html += (rainActive?"Active":"Off"); html += F("</b></div>");
  html += F("<div id='windBadge' class='badge "); html += (windActive ? "b-warn" : "b-ok"); html += F("'>Wind: <b>");
  html += (windActive?"Active":"Off"); html += F("</b></div>");
  html += F("<div class='badge'>Cause: <b id='rainCauseBadge'>"); html += causeText; html += F("</b></div>");
  html += F("<div class='badge'>1h (Now): <b id='acc1h'>--</b> mm</div>");
  html += F("<div class='badge'>24h (Total): <b id='acc24'>--</b> mm</div>");
  html += F("</div></div>");

  // Next Water
  html += F("<div class='card'><h3>Next Water</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:6px'>");
  html += F("<div class='chip'>Zone: <b id='nwZone'>--</b></div>");
  html += F("<div class='chip'>Start: <b id='nwTime'>--:--</b></div>");
  html += F("<div class='chip'>ETA: <b id='nwETA'>--</b></div>");
  html += F("<div class='chip'>Duration: <b id='nwDur'>--</b></div>");
  html += F("</div><div class='hint'>Active weather delays cancel watering if scheduled.</div></div>");

  html += F("</div></div>"); // end glass / grid

  // ---------- Schedules (collapsible) ----------
  static const char* DLBL[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  html += F("<div class='center'><div class='card sched'>");
  html += F("<h3 class='collapse' onclick=\"const b=document.getElementById('schedBody');"
            "const a=this.querySelector('.arr');"
            "const open=b.style.display!=='none';"
            "b.style.display=open?'none':'block';"
            "a.textContent=open?'>':'v';\">");
  html += F("<span class='sched-title'><span>Schedules</span><span class='sched-sub'>Edit by zone</span></span><span class='arr'>></span></h3>");
  html += F("<div id='schedBody' class='sched-ctr' style='display:none'>");
  html += F("<div class='sched-grid'>");

  for (int z=0; z<(int)zonesCount; ++z) {
    html += F("<div class='sched-card'><h4><span class='sched-badge'>Z");
    html += String(z + 1);
    html += F("</span> ");
    html += zoneNames[z];
    html += F("</h4><form method='POST' action='/submit'>");
    html += F("<input type='hidden' name='onlyZone' value='"); html += String(z); html += F("'>");

    // Name
    html += F("<div class='rowx'><label>Name</label><div class='field'>");
    html += F("<input class='in' type='text' name='zoneName"); html += String(z);
    html += F("' value='"); html += zoneNames[z]; html += F("' maxlength='32' style='flex:1;min-width:200px'>");
    html += F("</div></div>");

    // Start 1
    html += F("<div class='rowx'><label>Start 1</label><div class='field inline'>");
    html += F("<input class='in' type='number' min='0' max='23' name='startHour"); html += String(z);
    html += F("' value='"); html += String(startHour[z]); html += F("'>");
    html += F("<span class='sep'>:</span>");
    html += F("<input class='in' type='number' min='0' max='59' name='startMin"); html += String(z);
    html += F("' value='"); html += String(startMin[z]); html += F("'>");
    html += F("</div></div>");

    // Start 2
    html += F("<div class='rowx'><label>Start 2</label><div class='field inline'>");
    html += F("<input class='in' type='number' min='0' max='23' name='startHour2"); html += String(z);
    html += F("' value='"); html += String(startHour2[z]); html += F("'>");
    html += F("<span class='sep'>:</span>");
    html += F("<input class='in' type='number' min='0' max='59' name='startMin2"); html += String(z);
    html += F("' value='"); html += String(startMin2[z]); html += F("'>");
    html += F("<label class='toggle-inline'><input type='checkbox' name='enableStartTime2");
    html += String(z); html += F("' "); html += (enableStartTime2[z] ? "checked" : "");
    html += F("> Enable</label></div></div>");

    // Duration
    html += F("<div class='rowx'><label>Duration</label><div class='field inline'>");
    html += F("<input class='in' type='number' min='0' max='600' name='durationMin"); html += String(z);
    html += F("' value='"); html += String(durationMin[z]); html += F("'>");
    html += F("<span class='unit'>m</span><span class='sep'>:</span>");
    html += F("<input class='in' type='number' min='0' max='59' name='durationSec"); html += String(z);
    html += F("' value='"); html += String(durationSec[z]); html += F("'>");
    html += F("<span class='unit'>s</span></div></div>");

    // Duration 2 (used when Start 2 fires)
    html += F("<div class='rowx dur2row' id='dur2row"); html += String(z); html += F("' style='display:");
    html += (enableStartTime2[z] ? "grid" : "none");
    html += F("'><label>Duration 2</label><div class='field inline'>");
    html += F("<input class='in' type='number' min='0' max='600' name='duration2Min"); html += String(z);
    html += F("' value='"); html += String(duration2Min[z]); html += F("'>");
    html += F("<span class='unit'>m</span><span class='sep'>:</span>");
    html += F("<input class='in' type='number' min='0' max='59' name='duration2Sec"); html += String(z);
    html += F("' value='"); html += String(duration2Sec[z]); html += F("'>");
    html += F("<span class='unit'>s</span><small class='sub'>Used only for Start 2</small></div></div>");

    // Days
    html += F("<div class='rowx'><label>Days</label><div class='days-grid'>");
    for (int d=0; d<7; ++d) {
      html += F("<label class='day'><input type='checkbox' name='day"); html += String(z); html += "_"; html += String(d);
      html += F("' "); html += (days[z][d] ? "checked" : ""); html += F("> "); html += DLBL[d]; html += F("</label>");
    }
    html += F("</div></div>");

    // Actions
    html += F("<div class='toolbar' style='justify-content:flex-end'><button class='btn' type='submit'>Save Zone</button></div>");
    html += F("</form></div>");
  }

  html += F("</div>"); // .sched-grid
  html += F("<div class='toolbar' style='margin-top:10px;justify-content:flex-end'>"
            "<button class='btn' id='btn-save-all' title='Save all zone schedules'>Save All</button>"
            "</div>");
  html += F("</div></div></div>"); // #schedBody, .card.sched, .center

    // --- Live Zones ---
  html += F("<div class='center' style='margin-top:12px'><div class='card'>");
  html += F("<h3>Zones</h3><div class='zones'>");

  for (int z=0; z<(int)zonesCount; z++) {
    unsigned long rem  = 0;
    unsigned long total = (unsigned long)durationMin[z]*60 + durationSec[z];

    if (zoneActive[z]) {
      unsigned long elapsed = (millis() - zoneStartMs[z]) / 1000;
      rem = (elapsed < total ? total - elapsed : 0);
    }
    int pctDone = (zoneActive[z] && total>0)
                  ? (int)((100UL * (total - rem)) / total)
                  : 0;

    // Prettier duration label
    unsigned long durSec = total;
    unsigned int durM = durSec / 60;
    unsigned int durS = durSec % 60;

    html += F("<div class='zone-card'>");

    // Header: index + name + state badge
    html += F("<div class='zone-head'>");
      html += F("<div class='zone-title'>");
        html += F("<span class='zone-index'>Z");
        html += String(z+1);
        html += F("</span>");
        html += F("<span id='zone-");
        html += String(z);
        html += F("-dot' class='zone-dot ");
        html += (zoneActive[z] ? "on" : "");
        html += F("'></span>");
        html += F("<span class='big'>");
        html += zoneNames[z];
        html += F("</span>");
      html += F("</div>");
      html += F("<div id='zone-");
      html += String(z);
      html += F("-state' class='badge ");
      html += (zoneActive[z] ? "b-ok" : "");
      html += F("'>");
      html += (zoneActive[z] ? "Running" : "Off");
      html += F("</div>");
    html += F("</div>"); // .zone-head

    // Timer row: Remaining + bar
    html += F("<div class='zone-timer'>");

      html += F("<div class='zone-rem-wrap'>");
        html += F("<span class='zone-rem-label'>Remaining</span>");
        html += F("<div class='zone-rem' id='zone-");
        html += String(z);
        html += F("-rem'>");
        if (zoneActive[z]) {
          int rm = rem / 60;
          int rs = rem % 60;
          html += String(rm);
          html += F("m ");
          if (rs < 10) html += F("0");
          html += String(rs);
          html += F("s");
        } else {
          html += F("--");
        }
        html += F("</div>");
      html += F("</div>"); // .zone-rem-wrap

      html += F("<div class='zone-meter'>");
      html += F("<span class='zone-bar-label'>Progress</span>");
      html += F("<div class='meter'><div id='zone-");
      html += String(z);
      html += F("-bar' class='fill' style='width:");
      html += String(pctDone);
      html += F("%'></div></div>");
      html += F("</div>");

    html += F("</div>"); // .zone-timer

    // Meta row: Duration
    html += F("<div class='zone-meta-row'>");
      html += F("<span class='pill-soft'><span>Duration&nbsp;</span><b>");
      html += String(durM);
      html += F("m ");
      if (durS < 10) html += F("0");
      html += String(durS);
      html += F("s</b></span>");
      if (enableStartTime2[z]) {
        unsigned long d2 = (unsigned long)duration2Min[z]*60UL + (unsigned long)duration2Sec[z];
        unsigned int d2m = d2 / 60;
        unsigned int d2s = d2 % 60;
        html += F("<span class='pill-soft'><span>Duration 2&nbsp;</span><b>");
        html += String(d2m);
        html += F("m ");
        if (d2s < 10) html += F("0");
        html += String(d2s);
        html += F("s</b></span>");
      }
    html += F("</div>");

    // Actions
    html += F("<div class='toolbar zone-actions'>");
    html += F("<button type='button' class='btn' id='zone-");
    html += String(z);
    html += F("-on' onclick='toggleZone(");
    html += String(z);
    html += F(",1)'");
    if (zoneActive[z]) html += F(" disabled");
    html += F(">On</button>");
    html += F("<button type='button' class='btn btn-danger' id='zone-");
    html += String(z);
    html += F("-off' onclick='toggleZone(");
    html += String(z);
    html += F(",0)'");
    if (!zoneActive[z]) html += F(" disabled");
    html += F(">Off</button>");
    html += F("</div>"); // .toolbar

    html += F("</div>"); // .zone-card
  }

  html += F("</div></div></div>"); // Close zones block

  // --- Tools / System Controls ---
  html += F("<div class='grid center' style='margin:12px auto 20px'><div class='card' style='grid-column:1/-1'>");
  html += F("<h3>Tools</h3><div class='toolbar'>");
  html += F("<a class='btn' href='/setup'>Setup</a>");
  html += F("<a class='btn btn-secondary' href='/events'>Events</a>");
  html += F("<a class='btn btn-secondary' href='/status'>Status JSON</a>");
  html += F("</div></div></div>");

  html += F("<div class='grid center' style='margin:0 auto 24px'><div class='card' style='grid-column:1/-1'>");
  html += F("<h3>System Controls</h3><div class='toolbar'>");
  html += F("<button class='btn btn-secondary' id='toggle-backlight-btn' title='Invert OLED (night mode)'>LCD Toggle</button>");
  html += F("<button class='btn btn-danger' id='rebootBtn'>Reboot</button>");
  html += F("</div></div></div>");

  // --- JS ---
  html += F("<script>");
  html += F("function pad(n){return n<10?'0'+n:n;}");
  html += F("let _devEpoch=null; let _tickTimer=null; let _lastTemp=null;");
  html += F("function startDeviceClock(seedSec){_devEpoch=seedSec;if(_tickTimer)clearInterval(_tickTimer);");
  html += F("const draw=()=>{if(_devEpoch==null)return; const d=new Date(_devEpoch*1000);");
  html += F("const h=pad(d.getHours()),m=pad(d.getMinutes()),s=pad(d.getSeconds());");
  html += F("const el=document.getElementById('clock'); if(el) el.textContent=h+':'+m+':'+s; _devEpoch++;};");
  html += F("draw(); _tickTimer=setInterval(draw,1000);} ");
  html += F("function fmtClock12(epoch, offsetMin){");
  html += F(" if(typeof epoch!=='number' || epoch<=0) return '--:--';");
  html += F(" const t=new Date((epoch + (offsetMin||0)*60)*1000);");
  html += F(" let h=t.getUTCHours(); const m=t.getUTCMinutes(); const s=t.getUTCSeconds();");
  html += F(" const am=(h>=12)?'PM':'AM'; h=h%12; if(h===0) h=12;");
  html += F(" return pad(h)+':'+pad(m)+':'+pad(s)+' '+am;");
  html += F("} ");
  html += F("let _busy=false; async function postJson(url,payload){const body=payload?JSON.stringify(payload):\"{}\";");
  html += F("return fetch(url,{method:'POST',headers:{'Content-Type':'application/json','Cache-Control':'no-cache'},body});}");
  html += F("async function postForm(url, body){const opts={method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'}};");
  html += F("if(body)opts.body=body; return fetch(url,opts);} ");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-ghost,.pill').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("async function toggleZone(z,on){if(_busy)return;_busy=true;try{await postJson('/valve/'+(on?'on/':'off/')+z,{t:Date.now()});setTimeout(refreshStatus,200);}catch(e){console.error(e);}finally{_busy=false;}}");

  html += F("const btnBack=document.getElementById('toggle-backlight-btn');");
  html += F("if(btnBack){btnBack.addEventListener('click',async()=>{if(_busy)return;_busy=true;try{await postJson('/toggleBacklight',{t:Date.now()});}catch(e){}finally{_busy=false;}});} ");
  html += F("const btnReboot=document.getElementById('rebootBtn');");
  html += F("if(btnReboot){btnReboot.addEventListener('click',async()=>{if(confirm('Reboot controller now?')){try{await postJson('/reboot',{t:Date.now()});}catch(e){}}});} ");
  html += F("document.getElementById('btn-clear-delays')?.addEventListener('click',async()=>{await postForm('/clear_delays','a=1');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-24')?.addEventListener('click',async()=>{await postForm('/pause','sec=86400');setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-pause-7d')?.addEventListener('click',async()=>{await postForm('/pause','sec='+(7*86400));setTimeout(refreshStatus,200);});");
  html += F("document.getElementById('btn-resume')?.addEventListener('click',async()=>{await postForm('/resume','x=1');setTimeout(refreshStatus,200);});");

  // Master pill
  html += F("const btnMaster=document.getElementById('btn-master');");
  html += F("if(btnMaster){btnMaster.addEventListener('click',async()=>{");
  html += F("  const cur=(btnMaster.getAttribute('aria-pressed')==='true'); const turnOn=!cur;");
  html += F("  try{await fetch('/master',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:turnOn?'on=1':''});");
  html += F("      btnMaster.setAttribute('aria-pressed',turnOn?'true':'false');");
  html += F("      const st=document.getElementById('master-state'); if(st) st.textContent=turnOn?'On':'Off';");
  html += F("  }catch(e){console.error(e);} ");
  html += F("});}");

  // Theme
  html += F("function applyTheme(t){document.documentElement.setAttribute('data-theme',t==='dark'?'dark':'light');}");
  html += F("(function(){let saved=localStorage.getItem('theme');");
  html += F("if(saved!=='light'&&saved!=='dark'){saved=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';");
  html += F("localStorage.setItem('theme',saved);}applyTheme(saved);})();");
  html += F("document.getElementById('themeBtn')?.addEventListener('click',()=>{const cur=(document.documentElement.getAttribute('data-theme')==='dark')?'dark':'light';");
  html += F("const nxt=(cur==='dark')?'light':'dark';applyTheme(nxt);localStorage.setItem('theme',nxt);});");

  html += F("function toLocalHHMM(epoch){if(!epoch||epoch===0)return'--:--'; const d=new Date(epoch*1000); return pad(d.getHours())+':'+pad(d.getMinutes());}");
  html += F("async function refreshStatus(){try{const r=await fetch('/status');const st=await r.json();");
  html += F("if(typeof st.deviceEpoch==='number' && st.deviceEpoch>0 && _devEpoch===null){ startDeviceClock(st.deviceEpoch); }");
  html += F("const rb=document.getElementById('rainBadge');const wb=document.getElementById('windBadge');");
  html += F("if(rb){rb.className='badge '+(st.rainDelayActive?'b-bad':'b-ok');rb.innerHTML='Rain: <b>'+(st.rainDelayActive?'Active':'Off')+'</b>';}");
  html += F("if(wb){wb.className='badge '+(st.windDelayActive?'b-warn':'b-ok');wb.innerHTML='Wind: <b>'+(st.windDelayActive?'Active':'Off')+'</b>';}");
  html += F("const cause=document.getElementById('rainCauseBadge'); if(cause) cause.textContent=st.rainDelayCause||'Off';");
  html += F("const pct=st.tankPct||0; const tf=document.getElementById('tankFill'); const tl=document.getElementById('tankPctLabel');");
  html += F("if(tf) tf.style.width=Math.max(0,Math.min(100,pct))+'%'; if(tl) tl.textContent=pct+'%';");
  html += F("const src=document.getElementById('srcChip'); if(src) src.textContent=st.sourceMode||'';");
  html += F("const up=document.getElementById('upChip'); if(up) up.textContent=fmtClock12(st.deviceEpoch, st.utcOffsetMin);");
  html += F("const rssi=document.getElementById('rssiChip'); if(rssi) rssi.textContent=(st.rssi)+' dBm';");
  // NEW: Location chip + OpenWeatherMap link
  html += F("const cityEl=document.getElementById('cityName'); const cityLink=document.getElementById('owmLink');");
  html += F("if(typeof st.cityName==='string' && st.cityName.length){");
  html += F("  if(cityEl) cityEl.textContent=st.cityName;");
  html += F("  if(cityLink) cityLink.href='https://openweathermap.org/find?q='+encodeURIComponent(st.cityName);");
  html += F("}");




  // 1h & 24h
  html += F("var acc1h=document.getElementById('acc1h');");
  html += F("if(acc1h){var v=(typeof st.rain1hNow==='number')?st.rain1hNow:NaN;acc1h.textContent=isNaN(v)?'--':v.toFixed(1);}");
  html += F("const acc24=document.getElementById('acc24'); if(acc24){ const v=(typeof st.rain24hActual==='number')?st.rain24hActual:(typeof st.rain24h==='number'?st.rain24h:NaN); acc24.textContent=isNaN(v)?'--':v.toFixed(1);} ");

  html += F("if(Array.isArray(st.zones)){ st.zones.forEach((z,idx)=>{");
  html += F("const stateEl=document.getElementById('zone-'+idx+'-state'); const remEl=document.getElementById('zone-'+idx+'-rem'); const barEl=document.getElementById('zone-'+idx+'-bar'); const dotEl=document.getElementById('zone-'+idx+'-dot');");
  html += F("if(stateEl){stateEl.className='badge '+(z.active?'b-ok':'');stateEl.innerHTML=z.active?'Running':'Off';} if(dotEl){dotEl.className='zone-dot'+(z.active?' on':'');}");
  html += F("if(remEl){ if(z.active){ const r=z.remaining||0; const rm=Math.floor(r/60),rs=r%60; remEl.textContent=rm+'m '+(rs<10?'0':'')+rs+'s left'; } else remEl.textContent='--'; }");
  html += F("if(barEl){ let p=0; const total=z.totalSec||0; const rem=z.remaining||0; p=total>0?Math.max(0,Math.min(100,Math.round(100*(total-rem)/total))):0; barEl.style.width=p+'%'; }");
  html += F("const onBtn=document.getElementById('zone-'+idx+'-on'); const offBtn=document.getElementById('zone-'+idx+'-off');");
  html += F("if(onBtn) onBtn.disabled=!!z.active; if(offBtn) offBtn.disabled=!z.active;");
  html += F("}); }");

  // Weather stats
  html += F("const tmin=document.getElementById('tmin'); const tmax=document.getElementById('tmax'); const sunr=document.getElementById('sunr'); const suns=document.getElementById('suns'); const press=document.getElementById('press');");
  html += F("if(tmin) tmin.textContent=(st.tmin??0).toFixed(0);");
  html += F("if(tmax) tmax.textContent=(st.tmax??0).toFixed(0);");
  html += F("if(sunr) sunr.textContent = st.sunriseLocal || '--:--';");
  html += F("if(suns) suns.textContent = st.sunsetLocal  || '--:--';");
  html += F("if(press){ const p=st.pressure; press.textContent=(typeof p==='number' && p>0)?p.toFixed(0):'--'; }");
  html += F("const tempEl=document.getElementById('tempChip'); const trendEl=document.getElementById('tempTrend');");
  html += F("if(tempEl){ const v=st.temp; let arrow='\\u2192';");
  html += F("  if(typeof v==='number'){");
  html += F("    tempEl.textContent=v.toFixed(1)+' C';");
  html += F("    if(_lastTemp!==null){ const d=v-_lastTemp; if(d>0.1) arrow='\\u2191'; else if(d<-0.1) arrow='\\u2193'; }");
  html += F("    _lastTemp=v;");
  html += F("  } else { tempEl.textContent='--'; _lastTemp=null; }");
  html += F("  if(trendEl){ trendEl.textContent=arrow; trendEl.style.color=(arrow==='\\u2191')?'#16a34a':(arrow==='\\u2193')?'#dc2626':'inherit'; }");
  html += F("}");
  html += F("const feelsEl=document.getElementById('feelsChip'); if(feelsEl){ const v=st.feels_like; feelsEl.textContent=(typeof v==='number')?v.toFixed(1)+' C':'--'; }");
  html += F("const humEl=document.getElementById('humChip'); if(humEl){ const v=st.humidity; humEl.textContent=(typeof v==='number')?Math.round(v)+' %':'--'; }");
  html += F("const windEl=document.getElementById('windChip'); if(windEl){ const v=st.wind; windEl.textContent=(typeof v==='number')?v.toFixed(1)+' m/s':'--'; }");

  // keep master pill synced
  html += F("const bm=document.getElementById('btn-master'); const ms=document.getElementById('master-state');");
  html += F("if(bm && ms && typeof st.masterOn==='boolean'){ bm.setAttribute('aria-pressed', st.masterOn?'true':'false'); ms.textContent = st.masterOn?'On':'Off'; }");

  // Next Water
  html += F("(function(){ const zEl=document.getElementById('nwZone'); const tEl=document.getElementById('nwTime'); const eEl=document.getElementById('nwETA'); const dEl=document.getElementById('nwDur');");
  html += F("const epoch=st.nextWaterEpoch|0; const zone=st.nextWaterZone; const name=st.nextWaterName||(Number.isInteger(zone)?('Z'+(zone+1)):'--'); const dur=st.nextWaterDurSec|0;");
  html += F("function fmtDur(s){ if(s<=0) return '--'; const m=Math.floor(s/60), sec=s%60; return m+'m '+(sec<10?'0':'')+sec+'s'; }");
  html += F("function fmtETA(delta){ if(delta<=0) return 'now'; const h=Math.floor(delta/3600), m=Math.floor((delta%3600)/60); if(h>0) return h+'h '+m+'m'; return m+'m'; }");
  html += F("if(zEl) zEl.textContent=(zone>=0&&zone<255)?name:'--'; if(tEl) tEl.textContent=toLocalHHMM(epoch); if(dEl) dEl.textContent=fmtDur(dur);");
  html += F("let nowEpoch=(typeof st.deviceEpoch==='number'&&st.deviceEpoch>0&&_devEpoch!=null)?_devEpoch:Math.floor(Date.now()/1000);");
  html += F("if(eEl) eEl.textContent=epoch?fmtETA(epoch-nowEpoch):'--'; })();");

  html += F("}catch(e){} } setInterval(refreshStatus,1300); refreshStatus();");

  // expose zonesCount & Save All
  html += F("const ZC="); html += String(zonesCount); html += F(";");
  html += F("async function saveAll(){");
  html += F("  const fd=new URLSearchParams();");
  html += F("  for(let z=0; z<ZC; z++){");
  html += F("    const q=n=>document.querySelector(`[name='${n}']`);");
  html += F("    const add=(k)=>{const el=q(k); if(el){ if((el.type||'').toLowerCase()==='checkbox'){ if(el.checked) fd.append(k,'on'); } else { fd.append(k,el.value); } } };");
  html += F("    add('zoneName'+z); add('startHour'+z); add('startMin'+z); add('startHour2'+z); add('startMin2'+z); add('durationMin'+z); add('durationSec'+z); add('duration2Min'+z); add('duration2Sec'+z);");
  html += F("    add('enableStartTime2'+z);");
  html += F("    for(let d=0; d<7; d++) add('day'+z+'_'+d);");
  html += F("  }");
  html += F("  try{ await fetch('/submit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()}); location.reload(); }catch(e){ console.error(e); }");
  html += F("} ");
  html += F("document.getElementById('btn-save-all')?.addEventListener('click', saveAll);");
  // Toggle Duration 2 rows when Start 2 is enabled
  html += F("for(let z=0; z<ZC; z++){");
  html += F("  const cb=document.querySelector(`[name='enableStartTime2${z}']`);");
  html += F("  const row=document.getElementById('dur2row'+z);");
  html += F("  if(!cb||!row) continue;");
  html += F("  const sync=()=>{row.style.display=cb.checked?'grid':'none';};");
  html += F("  cb.addEventListener('change', sync); sync();");
  html += F("}");

  html += F("</script></body></html>");

  server.send(200, "text/html", html);
}

// Setup Page 
void handleSetupPage() {
  HttpScope _scope;
  loadConfig();
  String html; html.reserve(26000);

  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Setup - ESP32 Irrigation</title>");
  html += F("<style>");
  html += F("body{margin:0;font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;background:#0e1726;color:#e8eef6;font-size:15px}");
  html += F(".wrap{max-width:1100px;margin:28px auto;padding:0 16px}");
  html += F("h1{margin:0 0 16px 0;font-size:1.7em;letter-spacing:.3px;font-weight:800}");
  html += F(".card{background:#111927;border:1px solid #1f2a44;border-radius:16px;box-shadow:0 8px 34px rgba(0,0,0,.35);padding:18px 16px;margin-bottom:16px}");
  html += F(".card.narrow{max-width:960px;margin-left:auto;margin-right:auto}");
  html += F(".card h3{margin:0 0 12px 0;font-size:1.1em;font-weight:850;letter-spacing:.35px;color:#f2f6ff;");
  html += F("padding-bottom:6px;border-bottom:2px solid #1c74d9;display:flex;align-items:center;gap:8px}");
  html += F("label{display:inline-block;min-width:200px;font-size:.95rem;font-weight:600;color:#d6e1f4;text-align:left}");
  // Inputs + select share the same theme
  html += F("input[type=text],input[type=number],select{background:#0b1220;color:#e8eef6;border:1px solid #233357;border-radius:12px;padding:9px 12px;font-size:.95rem;text-align:left}");
  html += F("input[type=text],select{width:100%;max-width:520px}");
  html += F("input[type=number]{width:100%;max-width:200px}");
  html += F("input[type=text].in-wide,select.in-wide{max-width:600px}");
  html += F("input[type=text].in-med,select.in-med{max-width:440px}");
  html += F("input[type=number].in-xs{max-width:120px}");
  html += F("input[type=number].in-sm{max-width:160px}");
  html += F("input[type=number].in-md{max-width:240px}");
  html += F(".row{display:flex;align-items:center;gap:12px;margin:10px 0;flex-wrap:wrap}.row small{color:#9fb0ca;font-size:.85rem}");
  html += F(".btn{background:linear-gradient(180deg,#1c74d9,#165fba);color:#fff;border:1px solid rgba(0,0,0,.18);border-radius:12px;padding:10px 14px;font-weight:700;cursor:pointer;box-shadow:0 6px 16px rgba(25,118,210,.25);font-size:.95rem}");
  html += F(".btn-alt{background:#1b2537;color:#e8eef6;border:1px solid #2a3954;border-radius:12px;padding:10px 14px;font-size:.95rem}");
  html += F(".btn,.btn-alt{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active,.btn-alt:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn,.btn-alt{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".grid{display:grid;grid-template-columns:1fr;gap:14px}");
  html += F(".cols2{display:grid;grid-template-columns:1fr 1fr;gap:14px}");
  html += F("@media(max-width:760px){.cols2{grid-template-columns:1fr}label{min-width:150px}input[type=text].in-wide,select.in-wide,input[type=text].in-med,select.in-med{max-width:100%}}");
  html += F(".switchline{display:flex;gap:12px;align-items:center;flex-wrap:wrap}");
  html += F(".subhead{opacity:.85;margin:8px 0 6px 0;font-weight:700;font-size:.98rem}");
  html += F(".hr{height:1px;background:#1f2a44;margin:10px 0 8px 0;border:none}");

  // Chips + inline options for timezone mode
  html += F(".inline-options{display:flex;flex-wrap:wrap;gap:10px}");
  html += F(".chip{display:inline-flex;align-items:center;gap:6px;padding:6px 12px;border-radius:999px;background:#0b1220;border:1px solid #233357;font-size:.85rem;color:#e8eef6}");
  html += F(".chip input{margin:0}");
  html += F(".chip span{white-space:nowrap}");

  // Collapsible card styling
  html += F("details.collapse{padding:0;margin:0}");
  html += F("details.collapse summary{cursor:pointer;outline:none;list-style:none;font-weight:800;font-size:1.05rem;color:#e8eef6}");
  html += F("details.collapse summary::-webkit-details-marker{display:none}");
  html += F("details.collapse summary:after{content:'▸';margin-left:8px;transition:transform .18s ease;display:inline-block}");
  html += F("details.collapse[open] summary:after{transform:rotate(90deg)}");
  html += F(".collapse-body{margin-top:10px}");

  // Help text row: let the text wrap nicely
  html += F(".helptext label{min-width:0}");
  html += F(".helptext small{max-width:520px;display:block}");

  // Make native select look like themed dropdown with a chevron
  html += F("select{appearance:none;-webkit-appearance:none;-moz-appearance:none;padding-right:32px;");
  html += F("background-image:linear-gradient(45deg,transparent 50%,#9aa6c2 50%),linear-gradient(135deg,#9aa6c2 50%,transparent 50%);");
  html += F("background-position:calc(100% - 18px) 50%,calc(100% - 13px) 50%;background-size:5px 5px,5px 5px;background-repeat:no-repeat;}");
  html += F("body{-webkit-font-smoothing:antialiased;text-rendering:optimizeLegibility}");
  html += F(".card{transition:transform .12s ease,box-shadow .12s ease}");
  html += F(".card:hover{transform:translateY(-2px);box-shadow:0 12px 28px rgba(0,0,0,.22)}");
  html += F(".btn:hover,.btn-alt:hover{filter:brightness(1.06)}");
  html += F("input:focus-visible,select:focus-visible{outline:2px solid #1c74d9;outline-offset:2px}");

  // Desktop tuning
  html += F("@media(min-width:1024px){");
  html += F("body{font-size:16px;}");
  html += F(".wrap{max-width:1160px;padding:0 20px;}");
  html += F(".card{padding:20px 18px;}");
  html += F(".row{justify-content:flex-start;}");
  html += F("label{min-width:240px;}");
  html += F("input[type=text],select{max-width:560px;}");
  html += F("input[type=number]{max-width:220px;}");
  html += F("}");
  html += F("</style></head><body>");

  html += F("<div class='wrap'><h1>Setup</h1><form action='/configure' method='POST'>");

  // Weather
  html += F("<div class='card narrow'><h3>Weather</h3>");
  html += F("<div class='row'><label>API Key</label><input class='in-wide' type='text' name='apiKey' value='"); html += apiKey; html += F("'></div>");
  html += F("<div class='row'><label>City ID</label><input class='in-med' type='text' name='city' value='"); html += city; html += F("'><small>OpenWeatherMap city id</small></div>");
  html += F("</div>");

  // Zones
  html += F("<div class='card narrow'><h3>Zones</h3>");
  html += F("<div class='row'><label>Zone Count</label><input class='in-xs' type='number' min='1' max='");
  html += String(MAX_ZONES);
  html += F("' name='zonesMode' value='");
  html += String(zonesCount);
  html += F("'><small>Tank/Mains works with any zone count. Up to ");
  html += String(MAX_ZONES);
  html += F(" zones supported.</small></div>");

  // Run mode
  html += F("<div class='row switchline'><label>Run Mode</label>");
  html += F("<label><input type='checkbox' name='runConcurrent' "); html += (runZonesConcurrent ? "checked" : "");
  html += F("> Run Zones Together</label><small>Unchecked = One at a time</small></div>");
  html += F("</div>");

  // Tank (available for all modes; water source switching works with any zone count)
  html += F("<div class='card narrow'><h3>Tank & Water Source</h3>");
  html += F("<div class='row switchline'><label>Enable Tank</label><input type='checkbox' name='tankEnabled' ");
  html += (tankEnabled ? "checked" : "");
  html += F("><small>Unchecked = ignore tank level and force mains</small></div>");
  html += F("<div id='tankCard'>");

  html += F("<div class='row switchline'><label>Water Source</label>");

  // Auto
  html += F("<label><input type='radio' name='waterMode' value='auto' ");
  if (!justUseTank && !justUseMains) html += F("checked");
  html += F("> Auto (Tank + Mains)</label>");

  // Only Tank
  html += F("<label><input type='radio' name='waterMode' value='tank' ");
  if (justUseTank) html += F("checked");
  html += F("> Only Tank</label>");

  // Only Mains
  html += F("<label><input type='radio' name='waterMode' value='mains' ");
  if (justUseMains) html += F("checked");
  html += F("> Only Mains</label>");

  html += F("<small>");
  html += "Tank/mains switching works with any zone count";
  html += F("</small></div>");

  html += F("<div class='row'><label>Tank Low Threshold (%)</label>"
            "<input class='in-xs' type='number' min='0' max='100' name='tankThresh' value='");
  html += String(tankLowThresholdPct);
  html += F("'><small>Switch to mains if tank drops below this level</small></div>");

  html += F("<div class='row'><label>Tank Sensor GPIO</label><input class='in-xs' type='number' min='1' max='20' name='tankLevelPin' value='");
  html += String(tankLevelPin); html += F("'><small>ADC pin (ESP32-S3: GPIO1-20)</small></div>");
  html += F("<div class='row'><label></label><a class='btn-alt' href='/tank'>Calibrate Tank</a></div>");
  html += F("</div>"); // end tankCard
  html += F("</div>");

  // Physical rain & forecast
  html += F("<div class='card narrow'><h3>Rain Sources</h3>");
  html += F("<div class='row switchline'><label>Disable OWM Rain</label><input type='checkbox' name='rainForecastDisabled' ");
  html += (!rainDelayFromForecastEnabled ? "checked" : ""); html += F("><small>Checked = ignore OpenWeatherMap rain</small></div>");
  html += F("<div class='row switchline'><label>Enable Rain Sensor</label><input type='checkbox' name='rainSensorEnabled' "); html += (rainSensorEnabled?"checked":""); html += F("></div>");
  html += F("<div class='row'><label>Rain Sensor GPIO</label><input class='in-xs' type='number' min='0' max='39' name='rainSensorPin' value='"); html += String(rainSensorPin); html += F("'><small>e.g. 27</small></div>");
  html += F("<div class='row switchline'><label>Invert Sensor</label><input type='checkbox' name='rainSensorInvert' "); html += (rainSensorInvert?"checked":""); html += F("><small>Use if board is NO</small></div>");
  html += F("</div>");
  html += F("</div>");

      // Actions
  html += F("<div class='card narrow'><h3>Actions</h3>");
  html += F("<div class='row' style='gap:8px;flex-wrap:wrap'>");
  html += F("<button class='btn' type='submit'>Save</button>");
  html += F("<button class='btn-alt' formaction='/' formmethod='GET'>Home</button>");
  html += F("<button class='btn-alt' type='button' onclick=\"fetch('/clear_cooldown',{method:'POST'})\">Clear Cooldown</button>");
  html += F("</div></div>");

  // Delays & Pause + thresholds
  html += F("<div class='card narrow'><h3>Delays & Pause</h3>");
  html += F("<div class='cols2'>");

  // Left column: toggles
  html += F("<div>");
  html += F("<div class='subhead'>Delay Toggles</div><hr class='hr'>");
  html += F("<div class='row switchline'><label>Rain Delay</label><input type='checkbox' name='rainDelay' ");
  html += (rainDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>Wind Delay</label><input type='checkbox' name='windCancelEnabled' ");
  html += (windDelayEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row switchline'><label>System Pause</label><input type='checkbox' name='pauseEnable' ");
  html += (systemPaused ? "checked" : ""); html += F("><small>Enable System Pause</small></div>");
  html += F("<div class='row' style='gap:8px;flex-wrap:wrap'>");
  html += F("<button class='btn' type='button' id='btn-pause-24'>Pause 24h</button>");
  html += F("<button class='btn' type='button' id='btn-pause-7d'>Pause 7d</button>");
  html += F("<button class='btn' type='button' id='btn-resume'>Unpause</button>");
  html += F("<div class='row'><label>Pause for (hours)</label><input class='in-sm' type='number' min='0' max='720' name='pauseHours' value='");
  time_t nowEp = time(nullptr);
    {
    uint32_t remain = (pauseUntilEpoch > nowEp && systemPaused) ? (pauseUntilEpoch - nowEp) : 0;
    html += String(remain/3600);
  }
  html += F("'></div>");
  html += F("</div>");
  html += F("<div class='row'><label>LCD Brightness (%)</label><input class='in-xs' type='number' id='tftLevel' min='0' max='100' value='100'><button class='btn' type='button' id='btn-tft-bright'>Set</button></div>");
  html += F("</div>");

  // Right column: numeric thresholds / timers
  html += F("<div>");
  html += F("<div class='subhead'>Thresholds & Timers</div><hr class='hr'>");
  html += F("<div class='row'><label>Wind Threshold (m/s)</label><input class='in-sm' type='number' step='0.1' min='0' max='50' name='windSpeedThreshold' value='");
  html += String(windSpeedThreshold,1); html += F("'></div>");
  html += F("<div class='row'><label>Rain Cooldown (hours)</label><input class='in-sm' type='number' min='0' max='720' name='rainCooldownHours' value='");
  html += String(rainCooldownMin / 60);
  html += F("'><small>Cooldown period after rain stops</small></div>");
  html += F("<div class='row'><label>Rain Threshold 24h (mm)</label><input class='in-sm' type='number' min='0' max='200' name='rainThreshold24h' value='");
  html += String(rainThreshold24h_mm);
  html += F("'><small>Delay if >= threshold (24h total)</small></div>");
  html += F("</div>");

  html += F("</div>"); // end cols2
  html += F("</div>"); // end Delays card

  // GPIO fallback pins
  html += F("<div class='card narrow'><details class='collapse' ");
  html += (useGpioFallback ? "open" : "");
  html += F("><summary>GPIO Fallback (if I2C relays not found)</summary><div class='collapse-body'><div class='grid'>");
  for (uint8_t i=0;i<MAX_ZONES;i++){
    html += F("<div class='row'><label>Zone "); html += String(i+1);
    html += F(" Pin</label><input class='in-xs' type='number' min='-1' max='39' name='zonePin"); html += String(i);
    html += F("' value='"); html += String(zonePins[i]); html += F("'></div>");
  }
  html += F("<div class='row'><label></label><small>Use -1 to leave a zone unassigned. Zones above the 6 PCF channels use GPIO pins when set.</small></div>");
  html += F("<div class='row'><label>Main Pin</label><input class='in-xs' type='number' min='0' max='39' name='mainsPin' value='");
  html += String(mainsPin); html += F("'><small>GPIO relay for mains (used when GPIO mode or >4 zones)</small></div>");
  html += F("<div class='row'><label>Tank Relay Pin</label><input class='in-xs' type='number' min='0' max='39' name='tankPin' value='");
  html += String(tankPin); html += F("'><small>GPIO relay for tank (used when GPIO mode or >4 zones)</small></div>");

  // NEW: GPIO active polarity
  html += F("<div class='row switchline'><label>GPIO Active Low</label>");
  html += F("<input type='checkbox' name='gpioActiveLow' ");
  html += (gpioActiveLow ? "checked" : "");
  html += F(">");
  html += F("<small>Checked = LOW = ON (active-low relay modules). Unchecked = HIGH = ON.</small></div>");

  html += F("</div>");
  html += F("<div class='row' style='justify-content:flex-end;gap:8px'>");
  html += F("<button class='btn' type='submit'>Save</button>");
  html += F("</div>");
  html += F("</div></details></div>");

  // Manual buttons
  html += F("<div class='card narrow'><h3>Manual Buttons</h3>");
  html += F("<div class='row switchline'><label>Select Button Pin</label><input class='in-xs' type='number' min='-1' max='39' name='manualSelectPin' value='");
  html += String(manualSelectPin);
  html += F("'><small>-1 to disable. Uses INPUT_PULLUP; press = LOW.</small></div>");
  html += F("<div class='row switchline'><label>Start/Stop Button Pin</label><input class='in-xs' type='number' min='-1' max='39' name='manualStartPin' value='");
  html += String(manualStartPin);
  html += F("'><small>Toggles the selected zone on/off.</small></div>");
  html += F("<div class='row'><label>Selected Zone</label><div class='sub'>Z");
  html += String(manualSelectedZone + 1);
  html += F(" (cycles with Select button)</div></div>");
  html += F("</div>");

  // Timezone
  html += F("<div class='card narrow'><h3>Timezone</h3>");

  // Mode selector - cleaner row, better labels
  html += F("<div class='row switchline'>");
  html += F("<label>Mode</label>");
  html += F("<div class='inline-options'>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='0' ");
  html += (tzMode==TZ_POSIX ? "checked" : "");
  html += F(">");
  html += F("<span>POSIX string</span>");
  html += F("</label>");

  html += F("<label class='chip'>");
  html += F("<input type='radio' name='tzMode' value='2' ");
  html += (tzMode==TZ_FIXED ? "checked" : "");
  html += F(">");
  html += F("<span>Fixed offset</span>");
  html += F("</label>");

  html += F("</div></div>"); // end row + inline-options

  // POSIX string input
  html += F("<div class='row'>");
  html += F("<label>Timezone</label>");
  html += F("<input class='in-wide' type='text' name='tzPosix' value='");
  html += tzPosix;
  html += F("' placeholder='ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3'>");
  html += F("</div>");

  // Help text on its own row so it wraps nicely on mobile
  html += F("<div class='row helptext'>");
  html += F("<label></label>");
  html += F("<small>Example POSIX string with DST: ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3</small>");
  html += F("</div>");

  // IANA input + themed select
  html += F("<div class='row'><label>Select Timezone</label>");
  html += F("<div style='flex:1;display:grid;gap:6px'>");
  html += F("<input class='in-med' type='text' name='tzIANA' value='");
  html += tzIANA;
  html += F("' placeholder='Australia/Adelaide'>");
  html += F("<select class='in-med' id='tzIANASelect'><option value=''>Select from list</option></select>");
  html += F("</div>");
  html += F("</div>");

  html += F("<div class='row'><label>Fixed Offset (min)</label><input class='in-sm' type='number' name='tzFixed' value='");
  html += String(tzFixedOffsetMin);
  html += F("'><small>Minutes from UTC</small></div>");
  html += F("</div>"); // end Timezone card

  // MQTT
  html += F("<div class='card narrow'><h3>MQTT (Home Assistant)</h3>");
  html += F("<div class='row switchline'><label>Enable MQTT</label><input type='checkbox' name='mqttEnabled' "); html += (mqttEnabled ? "checked" : ""); html += F("></div>");
  html += F("<div class='row'><label>Broker Host</label><input class='in-wide' type='text' name='mqttBroker' value='"); html += mqttBroker; html += F("'></div>");
  html += F("<div class='row'><label>Port</label><input class='in-xs' type='number' name='mqttPort' value='"); html += String(mqttPort); html += F("'></div>");
  html += F("<div class='row'><label>User</label><input class='in-med' type='text' name='mqttUser' value='"); html += mqttUser; html += F("'></div>");
  html += F("<div class='row'><label>Password</label><input class='in-med' type='text' name='mqttPass' value='"); html += mqttPass; html += F("'></div>");
  html += F("<div class='row'><label>Base Topic</label><input class='in-med' type='text' name='mqttBase' value='"); html += mqttBase; html += F("'><small>e.g. espirrigation</small></div>");
  html += F("</div>");

  html += F("</form>");

  // quick-actions + timezone JS
  html += F("<script>");
  html += F("async function post(path, body){try{await fetch(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body});}catch(e){console.error(e)}}");
  html += F("const g=id=>document.getElementById(id);");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn,.btn-alt').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("g('btn-toggle-backlight')?.addEventListener('click',()=>post('/toggleBacklight','x=1'));");
  html += F("g('btn-pause-24')?.addEventListener('click',()=>post('/pause','sec=86400'));");
  html += F("g('btn-pause-7d')?.addEventListener('click',()=>post('/pause','sec='+(7*86400)));");
  html += F("g('btn-resume')?.addEventListener('click',()=>post('/resume','x=1'));");
  html += F("g('btn-tft-bright')?.addEventListener('click',async()=>{const v=g('tftLevel'); if(!v) return; let n=parseInt(v.value||'100'); if(isNaN(n)) n=100; n=Math.min(100,Math.max(0,n)); v.value=n; try{await post('/tft_brightness','level='+n);}catch(e){console.error(e)}});");

  // === Timezone loading from Nayarsystems posix_tz_db with fallback ===
  html += F("const TZ_DB_URL='https://raw.githubusercontent.com/nayarsystems/posix_tz_db/master/zones.json';");
  html += F("const tzInput=document.getElementsByName('tzIANA')[0]||null;");
  html += F("const tzPosixInput=document.getElementsByName('tzPosix')[0]||null;");
  html += F("const tzSel=g('tzIANASelect');");

  // Hard-coded fallback zones used if fetch fails
  html += F("const FALLBACK_ZONES={");
  html += F("'Australia/Adelaide':'ACST-9:30ACDT-10:30,M10.1.0/2,M4.1.0/3',");
  html += F("'Australia/Sydney':'AEST-10AEDT-11,M10.1.0/2,M4.1.0/3',");
  html += F("'UTC':'UTC0'");
  html += F("};");

  // Helper to populate the select + sync inputs from a map of { IANA: POSIX }
  html += F("function buildTzOptions(zones){");
  html += F(" if(!tzSel||!tzInput) return;");
  html += F(" tzSel.innerHTML='<option value=\"\">Select from list</option>';");

  html += F(" const names=Object.keys(zones).sort();");
  html += F(" names.forEach(name=>{");
  html += F("   const opt=document.createElement('option');");
  html += F("   opt.value=name;");
  html += F("   opt.textContent=name;");
  html += F("   if(tzInput.value===name) opt.selected=true;");
  html += F("   tzSel.appendChild(opt);");
  html += F(" });");

  // If tzInput already has a valid value, sync POSIX
  html += F(" if(tzInput.value && zones[tzInput.value] && tzPosixInput){");
  html += F("   tzPosixInput.value=zones[tzInput.value];");
  html += F(" }");

  // Try to guess browser zone if empty and available
  html += F(" if(!tzInput.value && window.Intl && Intl.DateTimeFormat){");
  html += F("   const guess=Intl.DateTimeFormat().resolvedOptions().timeZone;");
  html += F("   if(guess && zones[guess]){");
  html += F("     tzInput.value=guess;");
  html += F("     if(tzPosixInput) tzPosixInput.value=zones[guess];");
  html += F("     for(const opt of tzSel.options){");
  html += F("       if(opt.value===guess){opt.selected=true;break;}");
  html += F("     }");
  html += F("   }");
  html += F(" }");

  // When user picks a zone, update IANA + POSIX fields
  html += F(" tzSel.addEventListener('change',()=>{");
  html += F("   const val=tzSel.value;");
  html += F("   if(!val) return;");
  html += F("   tzInput.value=val;");
  html += F("   if(tzPosixInput && zones[val]) tzPosixInput.value=zones[val];");
  html += F(" });");
  html += F("}"); // end buildTzOptions

  // Tank card always shown now (no toggle)

  html += F("async function loadTimezones(){");
  html += F(" if(!tzSel||!tzInput) return;");

  html += F(" tzSel.innerHTML='<option value=\"\">Loading...</option>';");

  html += F(" try{");
  html += F("   const res=await fetch(TZ_DB_URL);");
  html += F("   if(!res.ok) throw new Error('HTTP '+res.status);");
  html += F("   const zones=await res.json();");
  html += F("   buildTzOptions(zones);");
  html += F(" }catch(e){");
  html += F("   console.error('tz load failed, using fallback',e);");
  html += F("   buildTzOptions(FALLBACK_ZONES);");
  html += F(" }");
  html += F("}");

  html += F("loadTimezones();");
  // === END TZ CODE ===

  html += F("</script>");

  html += F("</div></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Schedule POST (per-zone card or full form) ----------
void handleSubmit() {
  HttpScope _scope;

  // If onlyZone is present -> only update that zone from fields in this form
  if (server.hasArg("onlyZone")) {
    int z = server.arg("onlyZone").toInt();
    if (z >= 0 && z < (int)MAX_ZONES) {
      // Zone name
      if (server.hasArg("zoneName"+String(z))) {
        String nm=cleanName(server.arg("zoneName"+String(z)));
        if (nm.length()) zoneNames[z]=nm;
      }
      // Times / duration
      if (server.hasArg("startHour"+String(z)))  startHour[z]  = server.arg("startHour"+String(z)).toInt();
      if (server.hasArg("startMin"+String(z)))   startMin[z]   = server.arg("startMin"+String(z)).toInt();
      if (server.hasArg("startHour2"+String(z))) startHour2[z] = server.arg("startHour2"+String(z)).toInt();
      if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = server.arg("startMin2"+String(z)).toInt();
      if (server.hasArg("durationMin"+String(z))) durationMin[z]=server.arg("durationMin"+String(z)).toInt();
      if (server.hasArg("durationSec"+String(z))) durationSec[z]=server.arg("durationSec"+String(z)).toInt();
      if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=server.arg("duration2Min"+String(z)).toInt();
      if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=server.arg("duration2Sec"+String(z)).toInt();
      enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
      // Days
      for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));

      saveSchedule(); saveConfig();
      server.sendHeader("Location","/",true); server.send(302,"text/plain","");
      return;
    }
  }

  // Otherwise: legacy full update of all zones (expects all fields present)
  for (int z=0; z<(int)MAX_ZONES; z++) {
    for (int d=0; d<7; d++) days[z][d] = server.hasArg("day"+String(z)+"_"+String(d));
    if (server.hasArg("zoneName"+String(z))) {
      String nm=cleanName(server.arg("zoneName"+String(z)));
      if (nm.length()) zoneNames[z]=nm;
    }
    if (server.hasArg("startHour"+String(z)))  startHour[z]  = server.arg("startHour"+String(z)).toInt();
    if (server.hasArg("startMin"+String(z)))   startMin[z]   = server.arg("startMin"+String(z)).toInt();
    if (server.hasArg("startHour2"+String(z))) startHour2[z] = server.arg("startHour2"+String(z)).toInt();
    if (server.hasArg("startMin2"+String(z)))  startMin2[z]  = server.arg("startMin2"+String(z)).toInt();
    if (server.hasArg("durationMin"+String(z))) durationMin[z]=server.arg("durationMin"+String(z)).toInt();
    if (server.hasArg("durationSec"+String(z))) durationSec[z]=server.arg("durationSec"+String(z)).toInt();
    if (server.hasArg("duration2Min"+String(z))) duration2Min[z]=server.arg("duration2Min"+String(z)).toInt();
    if (server.hasArg("duration2Sec"+String(z))) duration2Sec[z]=server.arg("duration2Sec"+String(z)).toInt();
    enableStartTime2[z] = server.hasArg("enableStartTime2"+String(z));
  }
  saveSchedule(); saveConfig();
  server.sendHeader("Location","/",true); server.send(302,"text/plain","");
}

// ---------- Event Log Page ----------
void handleLogPage() {
  HttpScope _scope;
  File f = LittleFS.open("/events.csv","r");
  if (!f) {
    server.send(404,"text/plain","No event log");
    return;
  }

  String html; html.reserve(9000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Event Log</title>");
  html += F("<style>");
  html += F("body{font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;background:#0f1522;color:#e8eef6;margin:0;font-size:15px}");
  html += F("header{background:#13213a;color:#fff;text-align:center;padding:18px 0 12px;font-size:1.3em}");
  html += F(".wrap{max-width:1120px;margin:18px auto;padding:0 12px}");
  html += F(".toolbar{margin:10px 0;display:flex;flex-wrap:wrap;gap:10px}");
  html += F(".btn{padding:10px 14px;background:#1e2d4c;border-radius:12px;text-decoration:none;color:#fff;font-size:.95rem;border:none;cursor:pointer;display:inline-block}");
  html += F(".btn{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F(".btn-danger{background:#b93b3b}.btn-warn{background:#a15517}");
  html += F(".table-wrap{margin-top:10px;overflow-x:auto;border-radius:12px;border:1px solid #22314f}");
  html += F("table{width:100%;border-collapse:collapse;background:#0b1220;min-width:760px}");
  html += F("th,td{border:1px solid #22314f;padding:8px 8px;font-size:.95em;text-align:left;white-space:nowrap}");
  html += F("th{background:#172540;position:sticky;top:0;z-index:1}");
  html += F("td:last-child{white-space:normal;max-width:260px}");
  html += F("a{color:#a9cbff}");
  html += F("@media(max-width:760px){header{font-size:1.15em}.wrap{padding:0 6px}.btn{flex:1;text-align:center}}");
  html += F("</style></head><body>");

  html += F("<header>Irrigation Event Log</header><div class='wrap'>");
  html += F("<div class='toolbar'>");
  html += F("<a class='btn' href='/'>Home</a>");
  html += F("<a class='btn' href='/download/events.csv'>Download CSV</a>");
  html += F("<form style='display:inline' method='POST' action='/clearevents'>");
  html += F("<button class='btn btn-danger' type='submit'>Clear</button></form>");
  html += F("<form style='display:inline' method='POST' action='/stopall'>");
  html += F("<button class='btn btn-warn' type='submit'>Stop All</button></form>");
  html += F("</div>");

  html += F("<div class='table-wrap'><table><tr>");
  html += F("<th>Time</th><th>Zone</th><th>Event</th><th>Source</th><th>Rain Delay</th><th>Details</th></tr>");

  while (f.available()) {
    String line=f.readStringUntil('\n');
    if (line.length()<5) continue;

    int i1=line.indexOf(','), i2=line.indexOf(',',i1+1), i3=line.indexOf(',',i2+1), i4=line.indexOf(',',i3+1);
    int i5=line.indexOf(',',i4+1), i6=line.indexOf(',',i5+1), i7=line.indexOf(',',i6+1), i8=line.indexOf(',',i7+1), i9=line.indexOf(',',i8+1);

    String ts   = line.substring(0,i1);
    String zone = line.substring(i1+1,i2);
    String ev   = line.substring(i2+1,i3);
    String src  = line.substring(i3+1,i4);
    String rd   = line.substring(i4+1,i5);

    String temp =(i6>i5)?line.substring(i5+1,i6):"";
    String hum  =(i7>i6)?line.substring(i6+1,i7):"";
    String wind =(i8>i7)?line.substring(i7+1,i8):"";
    String cond =(i9>i8)?line.substring(i8+1,i9):"";
    String city =(i9>=0)?line.substring(i9+1):"";

    String details = (temp.length()
                      ? ("T="+temp+"C, H="+hum+"%, W="+wind+"m/s, "+cond+" @ "+city)
                      : "n/a");

    html += F("<tr><td>"); html += ts;
    html += F("</td><td>"); html += zone;
    html += F("</td><td>"); html += ev;
    html += F("</td><td>"); html += src;
    html += F("</td><td>"); html += rd;
    html += F("</td><td>"); html += details; html += F("</td></tr>");
  }
  f.close();

  html += F("</table></div></div>");
  html += F("<script>");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("</script></body></html>");
  server.send(200,"text/html",html);
}

// ---------- Tank Calibration Page ----------
void handleTankCalibration() {
  HttpScope _scope;
  int raw = isValidAdcPin(tankLevelPin) ? analogRead(tankLevelPin) : -1;
  int pct=tankPercent();

  String html; html.reserve(2000);
  html += F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Tank Calibration</title>");
  html += F("<style>body{font-family:'Trebuchet MS','Candara','Segoe UI',sans-serif;background:#0f1522;color:#e8eef6;margin:0;font-size:15px}");
  html += F(".wrap{max-width:600px;margin:34px auto;padding:0 16px}.card{background:#0b1220;border:1px solid #22314f;border-radius:14px;padding:20px 16px}");
  html += F("h2{margin:0 0 12px 0;font-size:1.4em;letter-spacing:.2px}");
  html += F(".btn{background:#1976d2;color:#fff;border:none;border-radius:12px;padding:10px 16px;font-weight:600;cursor:pointer;font-size:.95rem}.row{display:flex;gap:12px;justify-content:space-between;margin-top:12px}");
  html += F(".btn{transition:transform .06s ease,box-shadow .06s ease,filter .06s ease}");
  html += F(".btn:active{transform:translateY(1px);box-shadow:inset 0 2px 6px rgba(0,0,0,.25)}");
  html += F(".btn{position:relative;overflow:hidden}");
  html += F(".ripple{position:absolute;border-radius:999px;transform:scale(0);background:rgba(255,255,255,.35);animation:ripple .5s ease-out;pointer-events:none}");
  html += F("@keyframes ripple{to{transform:scale(3.2);opacity:0;}}");
  html += F("a{color:#a9cbff}</style></head><body><div class='wrap'><h2>Tank Calibration</h2><div class='card'>");

  html += F("<p>Raw: <b>"); html += String(raw); html += F("</b></p>");
  html += F("<p>Calibrated range: <b>"); html += String(tankEmptyRaw); html += F("</b> to <b>"); html += String(tankFullRaw); html += F("</b></p>");
  if (!isValidAdcPin(tankLevelPin)) {
    html += F("<p style='color:#f59e0b'>Invalid ADC pin. Set tank sensor GPIO to 1-20 (ESP32-S3) in Setup.</p>");
  } else {
    html += F("<p>Level: <b>"); html += String(pct); html += F("%</b></p>");
  }
  html += F("<div class='row'><form method='POST' action='/setTankEmpty'><button class='btn' type='submit'>Set Empty</button></form>");
  html += F("<form method='POST' action='/setTankFull'><button class='btn' type='submit'>Set Full</button></form></div>");
  html += F("<p><a href='/'>Home</a> | <a href='/setup'>Setup</a></p></div></div>");
  html += F("<script>");
  html += F("function addRipple(e){const t=e.currentTarget; if(t.disabled) return; const rect=t.getBoundingClientRect();");
  html += F("const size=Math.max(rect.width,rect.height); const x=(e.clientX|| (rect.left+rect.width/2)) - rect.left - size/2;");
  html += F("const y=(e.clientY|| (rect.top+rect.height/2)) - rect.top - size/2;");
  html += F("const r=document.createElement('span'); r.className='ripple'; r.style.width=size+'px'; r.style.height=size+'px';");
  html += F("r.style.left=x+'px'; r.style.top=y+'px'; const old=t.querySelector('.ripple'); if(old) old.remove(); t.appendChild(r);");
  html += F("setTimeout(()=>{r.remove();},520);}");
  html += F("document.querySelectorAll('.btn').forEach(el=>{el.addEventListener('pointerdown',addRipple);});");
  html += F("setTimeout(()=>location.reload(),2000);");
  html += F("</script></body></html>");

  server.send(200,"text/html",html);
}

// ---------- Config & Schedule ----------

static String _safeReadLine(File& f) {
  if (!f.available()) return String("");
  String s = f.readStringUntil('\n');
  s.trim();
  return s;
}

void loadConfig() {
  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  String s;

  // Legacy & core
  if ((s = _safeReadLine(f)).length()) apiKey = s;
  if ((s = _safeReadLine(f)).length()) city   = s;
  if ((s = _safeReadLine(f)).length()) tzOffsetHours = s.toFloat(); // legacy

  if ((s = _safeReadLine(f)).length()) rainDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) windSpeedThreshold   = s.toFloat();
  if ((s = _safeReadLine(f)).length()) windDelayEnabled     = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseTank          = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) justUseMains         = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) tankEmptyRaw         = s.toInt();
  if ((s = _safeReadLine(f)).length()) tankFullRaw          = s.toInt();
  if ((s = _safeReadLine(f)).length()) {
    int z = s.toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }
  if ((s = _safeReadLine(f)).length()) rainSensorEnabled = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) rainSensorPin     = s.toInt();
  if ((s = _safeReadLine(f)).length()) rainSensorInvert  = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int th = s.toInt();
    if (th >= 0 && th <= 100) tankLowThresholdPct = th;
  }
  if ((s = _safeReadLine(f)).length()) tankEnabled = (s.toInt() == 1);

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      if (p >= -1 && p <= 39) zonePins[i] = p;
    }
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (p >= 0 && p <= 39) mainsPin = p;
  }
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (p >= 0 && p <= 39) tankPin  = p;
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualSelectPin = (p >= -1 && p <= 39) ? p : -1;
    }
  }
  if (f.available()) {
    if ((s = _safeReadLine(f)).length()) {
      int p = s.toInt();
      manualStartPin = (p >= -1 && p <= 39) ? p : -1;
    }
  }

  // Zone names
  for (int i = 0; i < MAX_ZONES && f.available(); i++) {
    String nm = _safeReadLine(f);
    if (nm.length()) zoneNames[i] = nm;
  }

  // trailing (older "new" ones)
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainDelayFromForecastEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemPaused                 = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) pauseUntilEpoch             = (uint32_t)s.toInt(); }

  // Timezone block
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzMode            = (TZMode)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzPosix          = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzIANA           = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) tzFixedOffsetMin = (int16_t)s.toInt(); }

  // NEW trailing: master / cooldown / threshold / run mode / MQTT
  if (f.available()) { if ((s = _safeReadLine(f)).length()) systemMasterEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainCooldownMin     = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) rainThreshold24h_mm = s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) runZonesConcurrent  = (s.toInt() == 1); }

  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttEnabled = (s.toInt() == 1); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBroker  = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPort    = (uint16_t)s.toInt(); }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttUser    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttPass    = s; }
  if (f.available()) { if ((s = _safeReadLine(f)).length()) mqttBase    = s; }

  // ? NEW: relay polarity (optional trailing line, backwards compatible)
  if (f.available()) { 
    if ((s = _safeReadLine(f)).length()) 
      relayActiveHigh = (s.toInt() == 1);
  }

  // NEW: persisted GPIO polarity for fallback mode
  if (f.available()) {
    if ((s = _safeReadLine(f)).length())
      gpioActiveLow = (s.toInt() == 1);
  }

  // NEW: tank level sensor pin (ADC)
  if (f.available()) {
  if ((s = _safeReadLine(f)).length()) {
    int p = s.toInt();
    if (isValidAdcPin(p)) tankLevelPin = p;
  }
  }

  f.close();

  // Derive hours from minutes (for UI & cooldown logic)
  if (rainCooldownMin < 0) rainCooldownMin = 0;
  rainCooldownHours = (uint8_t)(rainCooldownMin / 60);
}

void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  if (zonesCount < 1) zonesCount = 1;
  if (zonesCount > MAX_ZONES) zonesCount = MAX_ZONES;

  // Core / legacy order
  f.println(apiKey);
  f.println(city);
  f.println(String(tzOffsetHours, 3));

  f.println(rainDelayEnabled ? 1 : 0);
  f.println(String(windSpeedThreshold, 3));
  f.println(windDelayEnabled ? 1 : 0);
  f.println(justUseTank ? 1 : 0);
  f.println(justUseMains ? 1 : 0);
  f.println(tankEmptyRaw);
  f.println(tankFullRaw);
  f.println(zonesCount);
  f.println(rainSensorEnabled ? 1 : 0);
  f.println(rainSensorPin);
  f.println(rainSensorInvert ? 1 : 0);
  f.println(tankLowThresholdPct);
  f.println(tankEnabled ? 1 : 0);

  // ? REMOVE this line from here (it breaks the alignment!)
  // f.println(relayActiveHigh ? 1 : 0);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zonePins[i]);
  f.println(mainsPin);
  f.println(tankPin);
  f.println(manualSelectPin);
  f.println(manualStartPin);

  for (int i = 0; i < MAX_ZONES; i++) f.println(zoneNames[i]);

  // trailing
  f.println(rainDelayFromForecastEnabled ? 1 : 0);
  f.println(systemPaused ? 1 : 0);
  f.println(pauseUntilEpoch);

  // Timezone
  f.println((int)tzMode);
  f.println(tzPosix);
  f.println(tzIANA);
  f.println(tzFixedOffsetMin);

  // Master / cooldown / threshold / run mode / MQTT
  // keep minutes & hours in sync
  rainCooldownMin   = (int)rainCooldownHours * 60;
  if (rainCooldownMin < 0) rainCooldownMin = 0;

  f.println(systemMasterEnabled ? 1 : 0);
  f.println(rainCooldownMin);
  f.println(rainThreshold24h_mm);
  f.println(runZonesConcurrent ? 1 : 0);

  f.println(mqttEnabled ? 1 : 0);
  f.println(mqttBroker);
  f.println(mqttPort);
  f.println(mqttUser);
  f.println(mqttPass);
  f.println(mqttBase);

  // ? NEW: relay polarity written as trailing line to match loadConfig()
  f.println(relayActiveHigh ? 1 : 0);
  // NEW: persist GPIO polarity for fallback mode
  f.println(gpioActiveLow ? 1 : 0);
  // NEW: tank level sensor pin (ADC)
  f.println(tankLevelPin);

  f.close();
}

void loadSchedule() {
  File f = LittleFS.open("/schedule.txt","r");
  if (!f) return;

  for (int i=0; i<MAX_ZONES; i++) {
    String line=f.readStringUntil('\n'); line.trim(); if (!line.length()) continue;

    int idx=0; int tcount=0; int tokens[32];
    while (idx < (int)line.length() && tcount < 32) {
      int nidx=line.indexOf(',', idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim();
      tokens[tcount++] = sv.toInt();
      idx = (nidx<(int)line.length()) ? nidx+1 : nidx;
    }
    auto tok=[&](int k,int def)->int{ return (k < tcount) ? tokens[k] : def; };

    startHour[i]    = tok(0, startHour[i]);
    startMin[i]     = tok(1, startMin[i]);
    startHour2[i]   = tok(2, startHour2[i]);
    startMin2[i]    = tok(3, startMin2[i]);
    durationMin[i]  = tok(4, durationMin[i]);
    durationSec[i]  = tok(5, durationSec[i]);
    duration2Min[i] = tok(6, durationMin[i]);
    duration2Sec[i] = tok(7, durationSec[i]);

    int enIdx = (tcount >= 9) ? 8 : 6;
    enableStartTime2[i] = (tok(enIdx, enableStartTime2[i]) == 1);

    int dayStart = enIdx + 1;
    for (int d=0; d<7; d++) {
      days[i][d] = (tok(dayStart + d, days[i][d]) == 1);
    }
  }
  f.close();
}

void saveSchedule() {
  File f = LittleFS.open("/schedule.txt","w");
  if (!f) return;
  for (int i=0; i<MAX_ZONES; i++) {
    f.print(startHour[i]);  f.print(',');
    f.print(startMin[i]);   f.print(',');
    f.print(startHour2[i]); f.print(',');
    f.print(startMin2[i]);  f.print(',');
    f.print(durationMin[i]);f.print(',');
    f.print(durationSec[i]);f.print(',');
    f.print(duration2Min[i]);f.print(',');
    f.print(duration2Sec[i]);f.print(',');
    f.print(enableStartTime2[i] ? '1' : '0');
    for (int d=0; d<7; d++){ f.print(','); f.print(days[i][d] ? '1' : '0'); }
    f.println();
  }
  f.close();
}

void handleConfigure() {
  HttpScope _scope;

    // NEW: snapshot current values before applying POST changes
  String oldApiKey = apiKey;
  String oldCity   = city;
  int oldZonePins[MAX_ZONES]; for (int i=0;i<MAX_ZONES;i++) oldZonePins[i] = zonePins[i];
  int oldMainsPin        = mainsPin;
  int oldTankPin         = tankPin;
  int oldTankLevelPin    = tankLevelPin;
  int oldManualSelectPin = manualSelectPin;
  int oldManualStartPin  = manualStartPin;
  int oldRainSensorPin   = rainSensorPin;
  bool pinsChanged       = false;

  // Weather
  if (server.hasArg("apiKey")) apiKey = server.arg("apiKey");
  if (server.hasArg("city"))   city   = server.arg("city");

  // Zones mode (1..MAX_ZONES, 4 keeps tank/mains behaviour)
  if (server.hasArg("zonesMode")) {
    int z = server.arg("zonesMode").toInt();
    if (z < 1) z = 1;
    if (z > (int)MAX_ZONES) z = MAX_ZONES;
    zonesCount = (uint8_t)z;
  }

  // Run mode (sequential vs concurrent)
  runZonesConcurrent = server.hasArg("runConcurrent");

  // Rain forecast gate
  rainDelayFromForecastEnabled = !server.hasArg("rainForecastDisabled");

  // Delay toggles
  rainDelayEnabled = server.hasArg("rainDelay");
  windDelayEnabled = server.hasArg("windCancelEnabled");

  // Physical rain sensor
  rainSensorEnabled = server.hasArg("rainSensorEnabled");
  if (server.hasArg("rainSensorPin")) {
    int pin = server.arg("rainSensorPin").toInt();
    if (pin >= 0 && pin <= 39) rainSensorPin = pin;
  }
  rainSensorInvert = server.hasArg("rainSensorInvert");

  // Wind threshold
  if (server.hasArg("windSpeedThreshold")) {
    windSpeedThreshold = server.arg("windSpeedThreshold").toFloat();
    if (windSpeedThreshold < 0) windSpeedThreshold = 0;
  }

  // Rain cooldown (hours ? minutes + uint8)
  if (server.hasArg("rainCooldownHours")) {
    int h = server.arg("rainCooldownHours").toInt();
    if (h < 0) h = 0;
    rainCooldownHours = (uint8_t)h;
    rainCooldownMin   = h * 60;
  }

  // 24h rain threshold (mm)
  if (server.hasArg("rainThreshold24h")) {
    int mm = server.arg("rainThreshold24h").toInt();
    if (mm < 0) mm = 0;
    rainThreshold24h_mm = mm;
  }

  // Pause handling
  bool resumeNow = server.hasArg("resumeNow");
  int pauseHours = server.hasArg("pauseHours") ? server.arg("pauseHours").toInt() : 0;
  time_t nowEp   = time(nullptr);

  if (resumeNow) {
    systemPaused    = false;
    pauseUntilEpoch = 0;
  } else {
    if (server.hasArg("pauseEnable")) {
      if (pauseHours > 0) {
        systemPaused    = true;
        pauseUntilEpoch = nowEp + (time_t)pauseHours * 3600;
      } else {
        // 0 hours + checkbox = "until manual resume"
        systemPaused    = true;
        pauseUntilEpoch = 0;
      }
    } else {
      // Checkbox not ticked ? clear pause
      systemPaused    = false;
      pauseUntilEpoch = 0;
    }
  }

  // Water mode
  if (server.hasArg("waterMode")) {
    String wm = server.arg("waterMode");
    if (wm == "tank") {
      justUseTank  = true;
      justUseMains = false;
    } else if (wm == "mains") {
      justUseTank  = false;
      justUseMains = true;
    } else { // auto
      justUseTank  = false;
      justUseMains = false;
    }
  }

  // Tank low threshold
  if (server.hasArg("tankThresh")) {
    int th = server.arg("tankThresh").toInt();
    if (th < 0)   th = 0;
    if (th > 100) th = 100;
    tankLowThresholdPct = th;
  }
  tankEnabled = server.hasArg("tankEnabled");

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    String key = "zonePin" + String(i);
    if (server.hasArg(key)) {
      int p = server.arg(key).toInt();
      if (p >= -1 && p <= 39) zonePins[i] = p;
    }
  }
  if (server.hasArg("mainsPin")) {
    int p = server.arg("mainsPin").toInt();
    if (p >= 0 && p <= 39) mainsPin = p;
  }
  if (server.hasArg("tankPin")) {
    int p = server.arg("tankPin").toInt();
    if (p >= 0 && p <= 39) tankPin = p;
  }
  if (server.hasArg("tankLevelPin")) {
    int p = server.arg("tankLevelPin").toInt();
    if (isValidAdcPin(p)) tankLevelPin = p;
  }
  if (server.hasArg("manualSelectPin")) {
    int p = server.arg("manualSelectPin").toInt();
    if ((p >= -1 && p <= 39)) manualSelectPin = p;
  }
  if (server.hasArg("manualStartPin")) {
    int p = server.arg("manualStartPin").toInt();
    if ((p >= -1 && p <= 39)) manualStartPin = p;
  }

  // Detect pin changes (needs to be done after all pin assignments above)
  for (int i=0;i<MAX_ZONES;i++) if (zonePins[i] != oldZonePins[i]) pinsChanged = true;
  if (mainsPin        != oldMainsPin)        pinsChanged = true;
  if (tankPin         != oldTankPin)         pinsChanged = true;
  if (tankLevelPin    != oldTankLevelPin)    pinsChanged = true;
  if (manualSelectPin != oldManualSelectPin) pinsChanged = true;
  if (manualStartPin  != oldManualStartPin)  pinsChanged = true;
  if (rainSensorPin   != oldRainSensorPin)   pinsChanged = true;

  // NEW: polarity setting
  gpioActiveLow = server.hasArg("gpioActiveLow"); 

  // Timezone mode + values
  if (server.hasArg("tzMode")) {
    int m = server.arg("tzMode").toInt();
    if (m < 0) m = 0;
    if (m > 2) m = 2;
    tzMode = (TZMode)m;
  }
  if (server.hasArg("tzPosix")) {
    String v = server.arg("tzPosix");
    v.trim();
    if (v.length()) tzPosix = v;
  }
  if (server.hasArg("tzIANA")) {
    String v = server.arg("tzIANA");
    v.trim();
    if (v.length()) tzIANA = v;
  }
  if (server.hasArg("tzFixed")) {
    tzFixedOffsetMin = (int16_t)server.arg("tzFixed").toInt();
  }

  // MQTT
  mqttEnabled = server.hasArg("mqttEnabled");
  if (server.hasArg("mqttBroker")) mqttBroker = server.arg("mqttBroker");
  if (server.hasArg("mqttPort"))   mqttPort   = (uint16_t)server.arg("mqttPort").toInt();
  if (server.hasArg("mqttUser"))   mqttUser   = server.arg("mqttUser");
  if (server.hasArg("mqttPass"))   mqttPass   = server.arg("mqttPass");
  if (server.hasArg("mqttBase"))   mqttBase   = server.arg("mqttBase");

  // Persist and re-apply runtime things
  saveConfig();
  initManualButtons();
    
    // --- NEW: if API key or City ID changed, force a weather refresh ---
  if (apiKey != oldApiKey || city != oldCity) {
    // Clear current / forecast caches and timers
    cachedWeatherData   = "";
    cachedForecastData  = "";
    lastWeatherUpdate   = 0;
    lastForecastUpdate  = 0;

    // Reset derived metrics so /status & UI show clean values until next fetch
    rain1hNow           = 0.0f;
    rain3hNow           = 0.0f;
    rainNext12h_mm      = 0.0f;
    rainNext24h_mm      = 0.0f;
    popNext12h_pct      = -1;
    nextRainIn_h        = -1;
    maxGust24h_ms       = 0.0f;
    todayMin_C          = NAN;
    todayMax_C          = NAN;
    todaySunrise        = 0;
    todaySunset         = 0;
    for (int i = 0; i < 24; ++i) {
      rainHist[i] = 0.0f;
    }
  }


  applyTimezoneAndSNTP();  // re-sync NTP with new TZ
  mqttSetup();             // reconfigure client; loop() will reconnect

  server.sendHeader("Location", "/setup", true);
  server.send(302, "text/plain", "");

  if (pinsChanged) {
    Serial.println("[CFG] Pin mapping changed, restarting to apply GPIO setup...");
    delay(200);
    ESP.restart();
  }
}

void handleClearEvents() {
  HttpScope _scope;
  if (LittleFS.exists("/events.csv")) {
    LittleFS.remove("/events.csv");
  }
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}


