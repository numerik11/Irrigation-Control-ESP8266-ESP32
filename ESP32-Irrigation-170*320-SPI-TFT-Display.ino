//130x230 SPI TFT 

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
#ifndef TFT_MISO
  #define TFT_MISO -1   // ST7789 typically doesn't use MISO
#endif
#define ENABLE_TFT 1
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
#include <PCF8574.h> // (Must use Library from Github)
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ST7789.h> // (Adafruit ST7735 and ST7789 Library)
#include <math.h>
extern "C" {
  #include "esp_log.h"
}
#include <time.h>
#include <ESPmDNS.h> 
#include <PubSubClient.h>   // MQTT

// ---------- Hardware ----------
static const uint8_t MAX_ZONES = 6;
constexpr uint8_t I2C_SDA = 4;
constexpr uint8_t I2C_SCL = 15;

TwoWire I2Cbus = TwoWire(0);
PCF8574 pcfIn (&I2Cbus, 0x22, I2C_SDA, I2C_SCL);
PCF8574 pcfOut(&I2Cbus, 0x24, I2C_SDA, I2C_SCL);

// ---------- ST7789 (1.9") TFT ----------
// Most 1.9" ST7789 modules are 170x320.
// If yours is 240x320, change TFT_W/TFT_H accordingly.
static constexpr int16_t TFT_W = 170;
static constexpr int16_t TFT_H = 320;

// Choose pins that do NOT clash with your I2C (4/15) or other IO.
// Example defaults (change to your wiring):
#define TFT_SCLK  18  // SCL
#define TFT_MOSI  23  // SDA
#define TFT_CS    5   // CS 
#define TFT_DC    13  // DC
#define TFT_RST   17  // RST
#define TFT_BL    32  // or -1 if tied to 3V3

SPIClass TFTSPI(VSPI);
Adafruit_ST7789 tft(&TFTSPI, TFT_CS, TFT_DC, TFT_RST);

// ===================== UI helpers + palette (MUST be before RainScreen/HomeScreen) =====================
static inline uint16_t RGB(uint8_t r, uint8_t g, uint8_t b){
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Palette (RGB565)
static const uint16_t C_BG     = ST77XX_BLACK;
static const uint16_t C_PANEL  = RGB(16, 22, 36);
static const uint16_t C_EDGE   = RGB(40, 55, 85);
static const uint16_t C_TEXT   = RGB(235, 242, 255);
static const uint16_t C_MUTED  = RGB(160, 175, 205);
static const uint16_t C_ACCENT = RGB(250, 210, 79);
static const uint16_t C_GOOD   = RGB(70, 200, 140);
static const uint16_t C_WARN   = RGB(255, 190, 70);
static const uint16_t C_BAD    = RGB(255, 110, 90);

static void drawCard(int x,int y,int w,int h,uint16_t fill,uint16_t edge){
  tft.fillRect(x, y, w, h, fill);
  tft.drawRect(x, y, w, h, edge);
  tft.drawPixel(x, y, fill); tft.drawPixel(x+w-1, y, fill);
  tft.drawPixel(x, y+h-1, fill); tft.drawPixel(x+w-1, y+h-1, fill);
}

static void drawTopBar(const char* title, const char* pill, uint16_t pillColor){
  const int W = tft.width();
  tft.fillRect(0, 0, W, 34, C_PANEL);
  tft.drawFastHLine(0, 34, W, C_EDGE);

  tft.setTextSize(2);
  tft.setTextColor(C_TEXT);
  tft.setCursor(8, 8);
  tft.print(title);

  int pillW = 8 + (int)strlen(pill) * 12; // rough for textSize(2)
  int x = W - pillW - 8;
  tft.fillRect(x, 7, pillW, 20, pillColor);
  tft.drawRect(x, 7, pillW, 20, C_EDGE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setCursor(x + 6, 9);
  tft.print(pill);
}

static void fmtMMSS(char* out, size_t n, unsigned long sec){
  unsigned long m = sec / 60;
  unsigned long s = sec % 60;
  snprintf(out, n, "%lum%02lus", m, s);
}

WiFiManager wifiManager;
WebServer server(80);
WiFiClient client;

// PCF mapping
const uint8_t ALL_P = 6;
const uint8_t PCH[ALL_P] = { P0, P1, P2, P3, P4, P5 };
uint8_t mainsChannel = P4; // 4-zone mode
uint8_t tankChannel  = P5; // 4-zone mode
uint8_t zonesCount   = 4;  // 4 or 6

// GPIO fallback (configurable polarity)
bool useGpioFallback = false;
bool gpioActiveLow   = false;
bool relayActiveHigh = true;

uint8_t zonePins[MAX_ZONES] = {18, 19, 12, 14, 25, 26};
uint8_t mainsPin = 25;
uint8_t tankPin  = 26;
int tankLevelPin = 36; // ADC1_CH0

const int LED_PIN  = 2;

// Physical rain sensor
bool rainSensorEnabled = false;
bool rainSensorInvert  = false;
int  rainSensorPin     = 15;

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
int lastCheckedMinute[MAX_ZONES] = { -1, -1, -1, -1, -1, -1 };

int tankEmptyRaw = 100;
int tankFullRaw  = 900;
String zoneNames[MAX_ZONES] = {"Zone 1","Zone 2","Zone 3","Zone 4","Zone 5","Zone 6"};

unsigned long zoneStartMs[MAX_ZONES] = {0};
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

int tankPercent() {
  const int N=8; uint32_t acc=0;
  for (int i=0;i<N;i++){ acc += analogRead(tankLevelPin); delayMicroseconds(200); }
  int raw=acc/N;
  int pct = map(raw, tankEmptyRaw, tankFullRaw, 0, 100);
  return constrain(pct, 0, 100);
}

bool isTankLow() {
  if (zonesCount == 6) return false;
  return tankPercent() <= tankLowThresholdPct;
}

bool physicalRainNowRaw() {
  if (!rainSensorEnabled) return false;
  pinMode(rainSensorPin, INPUT_PULLUP);
  int v = digitalRead(rainSensorPin); // LOW=dry, HIGH=wet (NC default)
  bool wet = (v == HIGH);
  if (rainSensorInvert) wet = !wet;
  return wet;
}

String sourceModeText() {
  if (zonesCount == 6) return "6-Zone";
  if (justUseTank)  return "Force:Tank";
  if (justUseMains) return "Force:Mains";
  return isTankLow() ? "Auto:Mains" : "Auto:Tank";
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
static const bool kUseStaticIP = true;
static const IPAddress kStaticIP(192, 168, 1, 123);
static const IPAddress kGateway(192, 168, 1, 1);
static const IPAddress kSubnet(255, 255, 255, 0);
static const IPAddress kDns(192, 168, 1, 1);

static void mdnsStart() {
  MDNS.end(); // in case it was running
  if (MDNS.begin(kHost)) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("arduino", "tcp", 3232); // keep OTA discoverable after restart
    Serial.println("[mDNS] started: http://espirrigation.local/");
  } else {
    Serial.println("[mDNS] begin() failed");
  }
}

// ---------- GPIO fallback helpers ----------
inline int gpioLevel(bool on) {
  if (gpioActiveLow) {
    // Active-LOW: LOW = ON, HIGH = OFF
    return on ? LOW : HIGH;
  } else {
    // Active-HIGH: HIGH = ON, LOW = OFF
    return on ? HIGH : LOW;
  }
}

inline void gpioInitOutput(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, gpioLevel(false));  // ensure OFF
}

inline void gpioZoneWrite(int z, bool on) {
  if (z < 0 || z >= (int)MAX_ZONES) return;
  digitalWrite(zonePins[z], gpioLevel(on));
}

inline void gpioSourceWrite(bool mainsOn, bool tankOn) {
  digitalWrite(mainsPin, gpioLevel(mainsOn));
  digitalWrite(tankPin, gpioLevel(tankOn));
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

static bool g_tftInvert = false;
static bool g_tftBlOn   = true;
static bool g_tftPwmReady = false;
static uint8_t g_tftBrightness = 125; // 0-255 duty when ON
static const int TFT_PWM_CH = 7;      // LEDC channel for TFT BL
static bool g_forceHomeReset = false; // force full HomeScreen repaint

static inline void tftBacklight(bool on){
  if (TFT_BL >= 0) {
    if (g_tftPwmReady) {
      ledcWrite(TFT_PWM_CH, on ? g_tftBrightness : 0);
    } else {
      pinMode(TFT_BL, OUTPUT);
      digitalWrite(TFT_BL, on ? HIGH : LOW);   // most modules: HIGH = on
    }
    g_tftBlOn = on;
  }
}

static void tftSetBrightness(uint8_t pct){ // pct: 0-100
  if (pct > 100) pct = 100;
  uint8_t duty = map(pct, 0, 100, 0, 255);
  g_tftBrightness = duty;
  if (TFT_BL >= 0) {
    if (g_tftPwmReady) {
      ledcWrite(TFT_PWM_CH, g_tftBlOn ? duty : 0);
    } else {
      // if no PWM, fall back to on/off at threshold
      digitalWrite(TFT_BL, (pct > 0) ? HIGH : LOW);
    }
  }
}
static void tftBegin(){
  // If you need custom pins, uncomment this and set TFT_SCK/TFT_MOSI:
  // SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);

  tft.init(TFT_W, TFT_H);
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tftBacklight(true);
}

static void tftHeader(const char* title){
  // simple header bar
  tft.fillRect(0, 0, TFT_W, 28, ST77XX_BLACK);
  tft.drawFastHLine(0, 28, TFT_W, ST77XX_WHITE);
  tft.setCursor(8, 8);
  tft.setTextSize(2);
  tft.print(title);
  tft.setTextSize(1);
}


// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

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
    checkI2CHealth();
  }

  pinMode(LED_PIN, OUTPUT);

  // ---------- ST7789 init ----------
TFTSPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  tft.init(TFT_W, TFT_H);
  tft.setRotation(3);                 // try 0/1/2/3 if orientation is wrong
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.setTextColor(ST77XX_WHITE);

  // Boot splash
  tft.setCursor(10, 20);
  tft.setTextSize(3);
  tft.print("Irrigation");

  tft.setTextSize(2);
  tft.setCursor(10, 70);
  tft.print("AP:");
  tft.setCursor(10, 95);
  tft.setTextSize(2);
  tft.print("ESPIrrigationAP");

  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.print("http://192.168.4.1");



  // Hostname + WiFi events
  WiFi.setHostname(kHost);

  // WiFiManager connect
  if (kUseStaticIP) {
    wifiManager.setSTAStaticIPConfig(kStaticIP, kGateway, kSubnet, kDns);
  }
  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    ESP.restart();
  }

  // Improve HTTP responsiveness
  WiFi.setSleep(false); // NEW: disable modem sleep for snappier responses

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.setCursor(10, 20);
  tft.print("Connected!");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print(WiFi.localIP().toString());

  tft.setCursor(10, 115);
  tft.print("espirrigation.local");

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
    ArduinoOTA.setHostname(kHost);
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
      if (zoneActive[i]) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000;
        unsigned long total   = (unsigned long)durationMin[i] * 60 + durationSec[i];
        rem = (elapsed < total ? total - elapsed : 0);
      }
      z["remaining"] = rem;
      z["totalSec"]  = (unsigned long)durationMin[i] * 60 + durationSec[i];
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
  server.on("/tft_brightness", HTTP_POST, [](){
    HttpScope _scope;
    if (TFT_BL < 0) { server.send(500,"text/plain","No backlight pin"); return; }
    int lvl = server.hasArg("level") ? server.arg("level").toInt() : 100;
    if (lvl < 0) lvl = 0; if (lvl > 100) lvl = 100;
    tftSetBrightness((uint8_t)lvl);
    server.send(200,"text/plain","OK");
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

  server.handleClient();           // serve UI first to keep it responsive
  wifiCheck();
  checkWindRain();
  mqttEnsureConnected();
  mqttPublishStatus();
  // Timed weather fetch after servicing HTTP to avoid blocking the UI
  tickWeather();
  tickManualButtons();

  // Track transitions to clear/refresh screens
  static bool lastWasDelayScreen = false;

  // Hard block while paused/master off/cooldown
  if (isBlockedNow()) {
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); lastWasDelayScreen = true; }
    delay(15); return;
  }

  const bool weatherDelay = (rainActive || windActive);
  if (weatherDelay) {
    for (int z=0; z<(int)zonesCount; ++z) if (zoneActive[z]) turnOffZone(z);
    if (now - lastScreenRefresh >= 1000) { lastScreenRefresh = now; RainScreen(); lastWasDelayScreen = true; }
  } else if (lastWasDelayScreen) {
    // Only clear once delay has actually ended
    lastWasDelayScreen = false;
    tft.fillScreen(C_BG);
    lastScreenRefresh = 0; // force immediate HomeScreen paint
    g_forceHomeReset = true;
  }

  if (now - lastTimeQuery >= TIME_QUERY_MS) {
    lastTimeQuery = now;
    time_t nowTime = time(nullptr);
    localtime_r(&nowTime, &cachedTm);
  }

  if (cachedTm.tm_hour == 0 && cachedTm.tm_min == 0) {
    if (!midnightDone) {
      memset(lastCheckedMinute, -1, sizeof(lastCheckedMinute));
      midnightDone = true;
    }
  } else midnightDone = false;

  if (!useGpioFallback && (now - lastI2cCheck >= I2C_CHECK_MS)) {
    lastI2cCheck = now; checkI2CHealth();
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
    }
  }

  bool manualActive = (manualScreenUntilMs != 0 && (int32_t)(manualScreenUntilMs - now) > 0);
  if (!manualActive && manualScreenUntilMs != 0) {
    manualScreenUntilMs = 0;
    g_forceHomeReset = true;
    lastScreenRefresh = 0;
  }

  if (!weatherDelay && now - lastScreenRefresh >= 1000) {
    lastScreenRefresh = now;
    if (manualActive) {
      drawManualSelection();
    } else {
      HomeScreen();
    }
  }
  delay(LOOP_SLEEP_MS);
}

// ---------- Connectivity / I2C health ----------
void wifiCheck() {
  if (WiFi.status()!=WL_CONNECTED) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect(); delay(600);
    if (!wifiManager.autoConnect("ESPIrrigationAP")) Serial.println("Reconnection failed.");
    else {
      Serial.println("Reconnected.");
      WiFi.setSleep(false); // keep disabled after reconnect
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
      Serial.println("I2C unstable ? GPIO fallback");
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

// ---------- Manual hardware buttons (select + start/stop) ----------
void showManualSelection() {
  manualScreenUntilMs = millis() + 15000UL;
  drawManualSelection();
}

void drawManualSelection() {
  const int W = tft.width();
  const int H = tft.height();

  tft.fillScreen(C_BG);
  drawTopBar("Manual", "BTN", C_ACCENT);

  int cardW = W - 24;
  int cardH = 90;
  int cardX = 12;
  int cardY = (H - cardH) / 2;
  drawCard(cardX, cardY, cardW, cardH, C_PANEL, C_EDGE);

  tft.setTextColor(C_TEXT);
  tft.setTextSize(3);
  tft.setCursor(cardX + 12, cardY + 16);
  tft.print("Zone ");
  tft.print((int)manualSelectedZone + 1);

  tft.setTextSize(1);
  tft.setTextColor(C_MUTED);
  tft.setCursor(cardX + 12, cardY + cardH - 18);
  tft.print("Press Start to toggle");
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
  bool needCur = (cachedWeatherData == "" ||
                  (nowms - lastWeatherUpdate >= weatherUpdateInterval));
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
      todayMin_C     = NAN; 
      todayMax_C     = NAN; 
      todaySunrise   = 0;   
      todaySunset    = 0;

      DynamicJsonDocument fc(14 * 1024);
      if (deserializeJson(fc, cachedForecastData) == DeserializationError::Ok) {
        if (fc["daily"].is<JsonArray>() && fc["daily"].size() > 0) {
          JsonObject d0 = fc["daily"][0];
          todayMin_C   = d0["temp"]["min"] | NAN;
          todayMax_C   = d0["temp"]["max"] | NAN;
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
          float hourlyMin = NAN;
          float hourlyMax = NAN;
          for (int i = 0; i < L24; i++) {
            JsonObject h = hr[i];
            float r = 0.0f;
            r += read1h(h["rain"]);
            r += read1h(h["snow"]);
            float t = h["temp"] | NAN;
            if (isfinite(t)) {
              if (!isfinite(hourlyMin) || t < hourlyMin) hourlyMin = t;
              if (!isfinite(hourlyMax) || t > hourlyMax) hourlyMax = t;
            }
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
          if (!isfinite(todayMin_C) && isfinite(hourlyMin)) todayMin_C = hourlyMin;
          if (!isfinite(todayMax_C) && isfinite(hourlyMax)) todayMax_C = hourlyMax;
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
    if (isnan(todayMin_C)) todayMin_C = cur["main"]["temp_min"] | NAN;
    if (isnan(todayMax_C)) todayMax_C = cur["main"]["temp_max"] | NAN;
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
void toggleBacklight(){
  if (TFT_BL < 0) return;
  static bool on = true;
  on = !on;
  digitalWrite(TFT_BL, on ? HIGH : LOW);
}

void updateLCDForZone(int zone) {
  static unsigned long lastUpdate=0; unsigned long now=millis();
  if (now - lastUpdate < 1000) return; lastUpdate = now;

  unsigned long elapsed=(now - zoneStartMs[zone]) / 1000;
  unsigned long total=(unsigned long)durationMin[zone]*60 + (unsigned long)durationSec[zone];
  unsigned long rem = (elapsed<total ? total - elapsed : 0);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(10, 20);
  tft.print(zoneNames[zone]);

  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.print("Elapsed: ");
  tft.print(elapsed/60);
  tft.print(":");
  if ((elapsed%60)<10) tft.print('0');
  tft.print(elapsed%60);

  tft.setCursor(10, 115);
  if (elapsed<total) {
    tft.print("Remain: ");
    tft.print(rem/60);
    tft.print("m ");
    if ((rem%60)<10) tft.print('0');
    tft.print(rem%60);
    tft.print("s");
  } else {
    tft.print("Complete");
  }
}

void RainScreen(){
  const int W = tft.width();

  // Avoid needless redraws to reduce flashing
  static bool lastPaused = false;
  static bool lastRainAct = false;
  static bool lastWindAct = false;
  static int  lastQueued = -1;
  static int  lastRunning = -1;
  static float lastRain24 = NAN;
  static float lastRain1h = NAN;
  static uint32_t lastCooldown = 0;
  static int lastRssi = 0;
  static bool lastMqtt = false;
  static String lastCause;
  static uint32_t lastDrawMs = 0;

  const bool paused = isPausedNow();
  const bool rainAct = rainActive;
  const bool windAct = windActive;
  String causeS = rainDelayCauseText();
  const char* cause = causeS.c_str();
  int queued=0, running=0;
  for (int i=0;i<(int)zonesCount;i++) {
    if (pendingStart[i]) queued++;
    if (zoneActive[i])   running++;
  }

  bool changed =
    paused != lastPaused ||
    rainAct != lastRainAct ||
    windAct != lastWindAct ||
    queued != lastQueued ||
    running != lastRunning ||
    fabsf(lastRainAmount - lastRain24) > 0.05f ||
    fabsf(rain1hNow - lastRain1h) > 0.05f ||
    rainCooldownUntilEpoch != lastCooldown ||
    causeS != lastCause;

  // Only consider RSSI/MQTT changes if they move meaningfully (avoid redraw on tiny RSSI jitter)
  bool rssiChanged = (abs(WiFi.RSSI() - lastRssi) >= 4);
  bool mqttChanged = (_mqtt.connected() != lastMqtt);

  uint32_t nowMs = millis();
  // Also allow a slow refresh every 8s for minor changes (including RSSI/MQTT)
  if (!changed && !rssiChanged && !mqttChanged && (nowMs - lastDrawMs) < 8000) return;

  lastPaused = paused;
  lastRainAct = rainAct;
  lastWindAct = windAct;
  lastQueued = queued;
  lastRunning = running;
  lastRain24 = lastRainAmount;
  lastRain1h = rain1hNow;
  lastCooldown = rainCooldownUntilEpoch;
  lastRssi = WiFi.RSSI();
  lastMqtt = _mqtt.connected();
  lastCause = causeS;
  lastDrawMs = nowMs;

  tft.fillScreen(C_BG);

  drawTopBar("RAIN / WIND", paused ? "PAUSED" : (rainActive||windActive?"DELAY":"READY"), paused ? C_WARN : (rainActive||windActive ? C_WARN : C_GOOD));

  int pad = 8;
  int y = 38; // start a bit higher for compact cards

  // Card 1: Rain history (24h + 1h)
  const int rainH = 60;
  drawCard(pad, y, W-2*pad, rainH, C_PANEL, C_EDGE);
  // Wind + rain summaries side by side
  float windNow = NAN;
  {
    DynamicJsonDocument js(384);
    DeserializationError derr = deserializeJson(js, cachedWeatherData);
    if (!derr) {
      windNow = js["wind"]["speed"] | NAN;
    }
  }

  const int colW = (W - 2 * pad) / 3;
  auto drawCol = [&](int idx, const char* lbl, const String& val){
    int x = pad + idx * colW + 4;
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(x, y + 4);
    tft.print(lbl);
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT);
    tft.setCursor(x, y + 24);
    tft.print(val);
  };

  drawCol(0, "1h",  String(rain1hNow,1) + "mm");
  drawCol(1, "24h", String(lastRainAmount,1) + "mm");
  {
    String w = isfinite(windNow) ? String(windNow,1) : String("--");
    w += "/";
    w += String(windSpeedThreshold,1);
    drawCol(2, "Wind", w);
  }

  y += rainH + 6;

  // Card 2: Cause / Wind / Cooldown / Master/Pause
  const int causeH = 84;
  drawCard(pad, y, W-2*pad, causeH, C_PANEL, C_EDGE);
  tft.setTextSize(2);
  tft.setTextColor(C_MUTED);
  tft.setCursor(pad+6, y+4);
  tft.print("Status");

  tft.setTextSize(2);
  tft.setTextColor(C_TEXT);
  tft.setCursor(pad+6, y+20);
  tft.print(cause ? cause : "--");

  // Wind + cooldown + pause/master rows
  float windNow2 = NAN;
  {
    DynamicJsonDocument js(384);
    DeserializationError derr = deserializeJson(js, cachedWeatherData);
    if (!derr) {
      windNow2 = js["wind"]["speed"] | NAN;
    }
  }

  tft.setTextSize(1);
  int lineY = y + 38;
  tft.setTextColor(C_MUTED);
  tft.setCursor(pad+6, lineY);
  tft.print("Cooldown: ");
  if (rainCooldownUntilEpoch > time(nullptr)) {
    uint32_t rem = (uint32_t)(rainCooldownUntilEpoch - time(nullptr));
    uint32_t mins = (rem + 59U) / 60U;
    if (mins >= 60U) { tft.print(mins/60); tft.print("h"); }
    else { tft.print(mins); tft.print("m"); }
  } else tft.print("None");

  lineY += 12;
  tft.setCursor(pad+6, lineY);
  tft.print("Pause ");
  tft.print(paused ? "Yes" : "No");
  tft.print(" | Master ");
  tft.print(systemMasterEnabled ? "On" : "Off");

  y += causeH + 6;

  // Card 3: Ops (running + queued + connectivity)
  const int opsH = 56;
  drawCard(pad, y, W-2*pad, opsH, C_PANEL, C_EDGE);
  tft.setTextSize(2);
  tft.setTextColor(C_MUTED);
  tft.setCursor(pad+6, y+4);
  tft.print("Ops");

  tft.setTextSize(2);
  tft.setTextColor(C_TEXT);
  tft.setCursor(pad+6, y+24);
  tft.print("Run ");
  tft.print(running);
  tft.setCursor(pad+6, y+42);
  tft.print("Q ");
  tft.print(queued);

  tft.setTextSize(1);
  tft.setTextColor(C_MUTED);
  tft.setCursor(W - pad - 70, y+18);
  tft.print("WiFi ");
  tft.print(WiFi.RSSI());
  tft.print("dBm");
  tft.setCursor(W - pad - 70, y+30);
  tft.print("MQTT ");
  tft.print(_mqtt.connected() ? "up" : "down");
}

void HomeScreen() {
  DynamicJsonDocument js(1024);
  (void)deserializeJson(js, cachedWeatherData);

  float temp = js["main"]["temp"] | NAN;
  int   hum  = js["main"]["humidity"] | -1;
  int   pct  = tankPercent();

  time_t now = time(nullptr);
  struct tm* tmv = localtime(&now);
  uint32_t nowMs = millis();

  const bool delayed = (rainActive || windActive || isPausedNow() || !systemMasterEnabled);

  const int W = tft.width();
  const int H = tft.height();

  const bool landscape = (H < W);

  // Common metrics
  const int pad = 6;
  const int gap = 6;
  const int topY = 44;

  // Header: big date + RSSI (no HOME/READY pill)
  static char lastTopDate[16] = "";
  static int  lastTopRssi = 9999;
  static int  lastTopMinute = -1;
  static uint32_t lastTopDrawMs = 0;
  char topDate[16] = "--/--/----";
  if (tmv) snprintf(topDate, sizeof(topDate), "%02d/%02d/%04d", tmv->tm_mday, tmv->tm_mon + 1, tmv->tm_year + 1900);
  int topRssi = WiFi.RSSI();
  int curMinute = tmv ? tmv->tm_min : -1;

  // Only redraw when date changes OR minute ticks OR RSSI changes notably (>=2dB) with a 15s debounce
  bool minuteChanged = (tmv && curMinute != lastTopMinute);
  bool dateChanged   = (strcmp(topDate, lastTopDate) != 0);
  bool rssiChanged   = (abs(topRssi - lastTopRssi) >= 2) && (nowMs - lastTopDrawMs >= 15000U);

  if (dateChanged || minuteChanged || rssiChanged) {
    tft.fillRect(0, 0, W, topY - 8, C_BG);
    tft.setTextSize(3);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(6, 6);
    tft.print(topDate);
    tft.setTextSize(2);
    tft.setTextColor(C_MUTED);
    char rssiTxt[16]; snprintf(rssiTxt, sizeof(rssiTxt), "%ddBm", topRssi);
    int16_t bx, by; uint16_t bw, bh;
    tft.getTextBounds(rssiTxt, 0, 0, &bx, &by, &bw, &bh);
    tft.setCursor(W - (int)bw - 6, 12);
    tft.print(rssiTxt);
    strncpy(lastTopDate, topDate, sizeof(lastTopDate));
    lastTopDate[sizeof(lastTopDate) - 1] = '\0';
    lastTopRssi = topRssi;
    lastTopMinute = curMinute;
    lastTopDrawMs = nowMs;
  }

  // -------- LANDSCAPE (H < W): two-column layout --------
  if (landscape) {
    static bool init = false;
    static bool lastDelayed = false;
    static int lastW = -1;
    static int lastH = -1;
    static int lastZonesCount = -1;
    static char lastTime[6] = "";
    static char lastDate[20] = "";
    static int lastTemp = -1000;
    static int lastHum = -2;
    static int lastPct = -1;
    static bool lastZoneState[MAX_ZONES] = {false};
    static uint32_t lastZoneRem[MAX_ZONES] = {0};
    if (g_forceHomeReset) {
      init = false; lastDelayed = false; lastW = lastH = -1; lastZonesCount = -1;
      lastTime[0] = lastDate[0] = '\0';
      lastTemp = -1000; lastHum = -2; lastPct = -1;
      for (int i=0;i<MAX_ZONES;i++){ lastZoneState[i]=false; lastZoneRem[i]=0; }
      g_forceHomeReset = false;
    }

    const int contentY = topY;
    const int bottomH = delayed ? 22 : 0;
    const int contentH = H - contentY - bottomH - 4;
    const int colGap = 8;

    const int leftW = 118;
    const int rightW = W - 2 * pad - colGap - leftW;
    const int leftX = pad;
    const int rightX = leftX + leftW + colGap;

    const bool layoutChanged = (!init || lastW != W || lastH != H || lastZonesCount != (int)zonesCount);
    const bool stateChanged = (delayed != lastDelayed);

    if (layoutChanged) {
      init = true;
      lastW = W;
      lastH = H;
      lastZonesCount = zonesCount;
      tft.fillScreen(C_BG);
    }
    lastDelayed = delayed;

    if (layoutChanged) {
      // Left: time card
      drawCard(leftX, contentY, leftW, contentH, C_PANEL, C_EDGE);

      // Right: weather/tank card
      const int statsH = 40;
      drawCard(rightX, contentY, rightW, statsH, C_PANEL, C_EDGE);

      // Right: zones grid
      const int zonesY = contentY + statsH + gap;
      const int zonesH = contentY + contentH - zonesY;
      drawCard(rightX, zonesY, rightW, zonesH, C_PANEL, C_EDGE);

      tft.setTextSize(1);
      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, zonesY + 4);
      tft.print("Zones");
    }

    // Time + date (top bar handles date; keep card for time + status lines)
    char tbuf[6] = "--:--";
    if (tmv) snprintf(tbuf, sizeof(tbuf), "%02d:%02d", tmv->tm_hour, tmv->tm_min);
    if (layoutChanged || strcmp(tbuf, lastTime) != 0) {
      tft.fillRect(leftX + 4, contentY + 4, leftW - 8, 28, C_PANEL);
      tft.setTextColor(C_ACCENT);
      tft.setTextSize(3);
      int16_t x1, y1; uint16_t tw, th;
      tft.getTextBounds(tbuf, 0, 0, &x1, &y1, &tw, &th);
      int timeY = contentY + 6;
      tft.setCursor(leftX + (leftW - (int)tw) / 2, timeY);
      tft.print(tbuf);
      strncpy(lastTime, tbuf, sizeof(lastTime));
      lastTime[sizeof(lastTime) - 1] = '\0';
    }

    // Next watering + status lines below the clock
    NextWaterInfo nw = computeNextWatering();
    char etaBuf[12] = "--:--";
    char zoneBuf[12] = "None";
    char remBuf[16] = "--";
    if (nw.zone >= 0) {
      snprintf(zoneBuf, sizeof(zoneBuf), "Z%d", nw.zone + 1);
      struct tm tnw;
      localtime_r(&nw.epoch, &tnw);
      strftime(etaBuf, sizeof(etaBuf), "%H:%M", &tnw);
      long diff = (long)difftime(nw.epoch, now);
      if (diff < 0) diff = 0;
      int hrs = (int)(diff / 3600L);
      int mins = (int)((diff % 3600L) / 60L);
      if (hrs > 0) {
        snprintf(remBuf, sizeof(remBuf), "in %dh%02dm", hrs, mins);
      } else {
        snprintf(remBuf, sizeof(remBuf), "in %dm", mins);
      }
    }

    const int nextY = contentY + 44; // push down away from clock
    const int nextH = 52;
    tft.fillRect(leftX + 4, nextY, leftW - 8, nextH, C_PANEL);
    tft.setTextSize(2);
    tft.setTextColor(C_TEXT);
    tft.setCursor(leftX + 8, nextY + 2);
    tft.print("Next ");
    tft.print(zoneBuf);
    tft.setCursor(leftX + 8, nextY + 20);
    tft.print(etaBuf);
    tft.setTextSize(1);
    tft.setTextColor(C_MUTED);
    tft.setCursor(leftX + 8, nextY + 38);
    tft.print(remBuf);

    tft.setTextColor(delayed ? C_WARN : C_MUTED);
    const int statY = nextY + nextH + 4;
    tft.fillRect(leftX + 4, statY, leftW - 8, 16, C_PANEL);
    tft.setTextSize(1);
    tft.setCursor(leftX + 8, statY + 2);
    if (delayed) {
      tft.print("Delay: ");
      String cS = rainDelayCauseText();
      tft.print(cS.c_str());
    } else {
      tft.print("Master ");
      tft.print(systemMasterEnabled ? "ON" : "OFF");
    }

    // Weather/tank
    int t0 = isnan(temp) ? -1000 : (int)lroundf(temp);
    if (layoutChanged || t0 != lastTemp || hum != lastHum || pct != lastPct) {
      const int statsH = 40;
      tft.fillRect(rightX + 4, contentY + 4, rightW - 8, statsH - 8, C_PANEL);

      tft.setTextSize(2);
      tft.setTextColor(C_TEXT);
      tft.setCursor(rightX + 8, contentY + 11); // push down for better centering
      tft.print("T:");
      if (isnan(temp)) tft.print("--");
      else { tft.print(temp, 0); tft.print("C"); }
      tft.print("  H:");
      if (hum < 0) tft.print("--");
      else { tft.print(hum); tft.print("%"); }

      tft.setTextColor(C_MUTED);
      tft.setCursor(rightX + 8, contentY + 27);
      
      lastTemp = t0;
      lastHum = hum;
      lastPct = pct;
    }

    // Zones grid
    const int statsH = 40;
    const int zonesY = contentY + statsH + gap;
    const int zonesH = contentY + contentH - zonesY;
    const int gridY = zonesY + 14;
    const int gridH = zonesY + zonesH - gridY - 6;
    const int cols = 2;
    int colW = (rightW - gap) / cols;
    int rows = (zonesCount + 1) / 2;          // two columns
    if (rows < 1) rows = 1;
    int rowH = gridH / rows;
    if (rowH < 18) rowH = 18;                 // even tighter rows

    for (int i = 0; i < (int)zonesCount && i < 6; i++) {
      int r = i / 2;
      int c = i % 2;
      int x = rightX + 6 + c * colW;
      int y = gridY + r * rowH;
      if (y + rowH > zonesY + zonesH) break;

      bool on = zoneActive[i];
      uint32_t rem = 0;
      if (on) {
        unsigned long elapsed = (millis() - zoneStartMs[i]) / 1000UL;
        unsigned long total = (unsigned long)durationMin[i] * 60UL + (unsigned long)durationSec[i];
        rem = (elapsed < total ? total - elapsed : 0UL);
      }

      bool stateChanged = (on != lastZoneState[i]);
      bool remChanged = (rem != lastZoneRem[i]);

      if (layoutChanged || stateChanged || remChanged) {
        // Clear full cell to prevent overlap artifacts
        tft.fillRect(x - 2, y - 1, colW - 4, rowH - 1, C_PANEL);

        // Single-line layout: Z#, pill, timer on the right
        tft.setTextSize(1);
        tft.setTextColor(C_TEXT);
        tft.setCursor(x, y + 2);
        tft.print("Z"); tft.print(i + 1);

        int pillX = x + 16;
        int pillY = y + 1;
        int pillW = 22;
        int pillH = 12;
        tft.fillRect(pillX, pillY, pillW, pillH, on ? C_GOOD : C_EDGE);
        tft.drawRect(pillX, pillY, pillW, pillH, C_EDGE);
        tft.setTextColor(on ? ST77XX_BLACK : C_TEXT);
        tft.setCursor(pillX + 4, pillY + 2);
        tft.print(on ? "ON" : "OFF");

        // Timer or placeholder at right
        char rbuf[10] = "--";
        if (on) { fmtMMSS(rbuf, sizeof(rbuf), rem); }
        tft.setTextColor(on ? C_TEXT : C_MUTED);
        int16_t bx, by; uint16_t bw, bh;
        tft.getTextBounds(rbuf, 0, 0, &bx, &by, &bw, &bh);
        // Position timer just to the right of the pill with a small gap
        int tx = pillX + pillW + 4;
        if (tx + (int)bw > x + colW - 4) tx = x + colW - 4 - (int)bw; // clamp inside cell
        tft.setCursor(tx, y + 2);
        tft.print(rbuf);

        lastZoneState[i] = on;
        lastZoneRem[i] = rem;
      }
    }

    // Bottom banner update
    if (layoutChanged || stateChanged) {
      if (delayed) {
        String cS = rainDelayCauseText();
        const char* c = cS.c_str();
        const int bh = 22;
        tft.fillRect(0, H - bh, W, bh, RGB(24, 18, 8));
        tft.drawFastHLine(0, H - bh, W, C_EDGE);
        tft.setTextSize(1);
        tft.setTextColor(C_WARN);
        tft.setCursor(6, H - bh + 6);
        tft.print("Delay:");
        tft.setTextColor(C_TEXT);
        tft.setCursor(52, H - bh + 6);
        tft.print((c && c[0]) ? c : "--");
      } else if (!layoutChanged) {
        tft.fillRect(0, H - 22, W, 22, C_BG);
      }
    }

    return;
  }
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
  if (durationMin[zone] == 0 && durationSec[zone] == 0) return false; // zero duration

  const bool match1 = (hr == startHour[zone]  && mn == startMin[zone]);
  const bool match2 = (enableStartTime2[zone] && hr == startHour2[zone] && mn == startMin2[zone]);

  if (match1 || match2) { lastCheckedMinute[zone] = mn; return true; }
  return false;
}

bool hasDurationCompleted(int zone) {
  unsigned long elapsed=(millis()-zoneStartMs[zone])/1000;
  unsigned long total=(unsigned long)durationMin[zone]*60 + (unsigned long)durationSec[zone];
  return (elapsed >= total);
}

void turnOnZone(int z) {
  checkWindRain();
  Serial.printf("[VALVE] Request ON Z%d rain=%d wind=%d blocked=%d\n",
                z+1, rainActive?1:0, windActive?1:0, isBlockedNow()?1:0);

  if (isBlockedNow())  { cancelStart(z, "BLOCKED", false); return; }
  if (rainActive)      { cancelStart(z, "RAIN",    true ); return; }
  if (windActive)      { cancelStart(z, "WIND",    false); return; }

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
  const char* src = "None";

  if (useGpioFallback) {
  // Use config for active level
  gpioZoneWrite(z, true);   // ON

  if (zonesCount == 4) {
    if (justUseTank) {
      src = "Tank";
      gpioSourceWrite(false, true);   // mains OFF, tank ON
    } else if (justUseMains) {
      src = "Mains";
      gpioSourceWrite(true, false);   // mains ON, tank OFF
    } else if (isTankLow()) {
      src = "Mains";
      gpioSourceWrite(true, false);   // mains ON, tank OFF
    } else {
      src = "Tank";
      gpioSourceWrite(false, true);   // mains OFF, tank ON
    }
  }
} else {
    // PCF8574 path unchanged (still active-LOW via expander)
    pcfOut.digitalWrite(PCH[z], LOW); // active-LOW
    if (zonesCount == 4) {
      if (justUseTank) {
        src = "Tank";
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel,  LOW);
      } else if (justUseMains) {
        src = "Mains";
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel,  HIGH);
      } else if (isTankLow()) {
        src = "Mains";
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel,  HIGH);
      } else {
        src = "Tank";
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel,  LOW);
      }
    }
  }

  logEvent(z, "START", src, false);

#if ENABLE_TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(2, 0);
  tft.print(zoneNames[z]);
  tft.print(" ON");
  delay(1000);
#endif

  // Force a clean redraw of Home after the ON banner
  g_forceHomeReset = true;
  tft.fillScreen(C_BG);
  HomeScreen();
}

void turnOffZone(int z) {
  Serial.printf("[VALVE] Request OFF Z%d\n", z+1);
  const char* src = "None";
  if (zonesCount == 4)
    src = justUseTank ? "Tank"
         : justUseMains ? "Mains"
         : isTankLow() ? "Mains"
         : "Tank";

  bool wasDelayed = rainActive || windActive || isPausedNow() ||
                    !systemMasterEnabled ||
                    (rainCooldownUntilEpoch > time(nullptr));
  logEvent(z, "STOPPED", src, wasDelayed);

  if (useGpioFallback) {
  // OFF everything in fallback according to config
  gpioZoneWrite(z, false);       // OFF
  if (zonesCount == 4) {
    gpioSourceWrite(false, false);  // mains OFF, tank OFF
  }
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], HIGH);
    if (zonesCount == 4) {
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel,  HIGH);
    }
  }

  zoneActive[z] = false;

#if ENABLE_TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(2, 0);
  tft.print(zoneNames[z]);
  tft.print(" OFF");
  delay(1000);
#endif

  // Ensure the TFT returns to the refreshed Home screen after showing the OFF banner
  g_forceHomeReset = true;
  HomeScreen();
}

static void printRight(int xRight, int y, const char* s){
  int16_t x1,y1; uint16_t w,h;
  tft.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  tft.setCursor(xRight - (int)w, y);
  tft.print(s);
}

void turnOnValveManual(int z) {
  if (z >= zonesCount) return;
  if (zoneActive[z])   return;

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
  const char* src = "None";

  if (useGpioFallback) {
  gpioZoneWrite(z, true);   // ON

  if (zonesCount == 4) {
    if (justUseTank) {
      src = "Tank";
      gpioSourceWrite(false, true);
    } else if (justUseMains) {
      src = "Mains";
      gpioSourceWrite(true, false);
    } else if (isTankLow()) {
      src = "Mains";
      gpioSourceWrite(true, false);
    } else {
      src = "Tank";
      gpioSourceWrite(false, true);
    }
  }
} else {
    // PCF8574 path unchanged
    pcfOut.digitalWrite(PCH[z], LOW);
    if (zonesCount == 4) {
      if (justUseTank) {
        src = "Tank";
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel,  LOW);
      } else if (justUseMains) {
        src = "Mains";
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel,  HIGH);
      } else if (isTankLow()) {
        src = "Mains";
        pcfOut.digitalWrite(mainsChannel, LOW);
        pcfOut.digitalWrite(tankChannel,  HIGH);
      } else {
        src = "Tank";
        pcfOut.digitalWrite(mainsChannel, HIGH);
        pcfOut.digitalWrite(tankChannel,  LOW);
      }
    }
  }

  logEvent(z, "MANUAL START", src, false);
}

void turnOffValveManual(int z) {
  if (z >= MAX_ZONES) return;
  if (!zoneActive[z]) return;

  // Turn this zone OFF
  if (useGpioFallback) {
    // Use configured polarity for GPIO fallback
    gpioZoneWrite(z, false);          // OFF
  } else {
    // PCF8574 path: keep active-LOW semantics (HIGH = OFF)
    pcfOut.digitalWrite(PCH[z], HIGH);
  }

  zoneActive[z] = false;

  bool anyStillOn = false;
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) {
      anyStillOn = true;
      break;
    }
  }

  // If no zones left on and we are in 4-zone (tank/mains) mode, turn both feed relays OFF
  if (!anyStillOn && zonesCount == 4) {
    if (useGpioFallback) {
      // Respect configured polarity for mains/tank
      gpioSourceWrite(false, false);   // mains OFF, tank OFF
    } else {
      // PCF8574: HIGH = OFF on both channels
      pcfOut.digitalWrite(mainsChannel, HIGH);
      pcfOut.digitalWrite(tankChannel,  HIGH);
    }
  }
}

// ---------- Next Water (queue-first) ----------
static NextWaterInfo computeNextWatering() {
  NextWaterInfo best{0, -1, 0};

  for (int z=0; z<(int)zonesCount; ++z) {
    if (pendingStart[z]) {
      best.epoch = time(nullptr);
      best.zone  = z;
      best.durSec = (uint32_t)durationMin[z]*60u + (uint32_t)durationSec[z];
      return best;
    }
  }

  time_t now = time(nullptr);
  struct tm base; localtime_r(&now, &base);

  auto consider = [&](int z, int hr, int mn, bool enabled) {
    if (!enabled) return;
    if (durationMin[z] == 0 && durationSec[z] == 0) return;

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
          best.durSec = (uint32_t)durationMin[z]*60u + (uint32_t)durationSec[z];
        }
        return;
      }
      t += 24*60*60;
    }
  };

  for (int z=0; z<(int)zonesCount; ++z) {
    consider(z, startHour[z],  startMin[z],  true);
    consider(z, startHour2[z], startMin2[z], enableStartTime2[z]);
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
  float temp = NAN, hum = NAN, ws = NAN;
  if (!werr) {
    if (js["main"]["temp"].is<float>())     temp = js["main"]["temp"].as<float>();
    if (js["main"]["humidity"].is<float>()) hum  = js["main"]["humidity"].as<float>();
    if (js["wind"]["speed"].is<float>())    ws   = js["wind"]["speed"].as<float>();
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
  html += F(".card h3{margin:0 0 8px 0;font-size:1.15rem;color:var(--ink);font-weight:800;letter-spacing:.3px}");
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
  html += F("<div class='chip'>Temp "); html += (isnan(temp) ? String("--") : String(temp,1)+" C"); html += F("</div>");
  html += F("<div class='chip'>Humidity ");  html += (isnan(hum)  ? String("--") : String((int)hum)+" %"); html += F("</div>");
  html += F("<div class='chip'>Wind "); html += (isnan(ws)   ? String("--") : String(ws,1)+" m/s"); html += F("</div>");
  html += F("<div class='chip'>Condition <b id='cond'>");
  html += cond.length() ? cond : String("--");
  html += F("</b></div></div></div>");

  html += F("<div class='card'><h3>Weather Today</h3>");
  html += F("<div class='grid' style='grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:6px'>");
  html += F("<div class='chip'><b>Low</b>&nbsp;<b id='tmin'>---</b> C</div>");
  html += F("<div class='chip'><b>High</b>&nbsp;<b id='tmax'>---</b> C</div>");
  html += F("<div class='chip'><b>Pressure</b>&nbsp;<b id='press'>--</b></div>");
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
  html += F("<a class='btn btn-secondary' href='/tank'>Calibrate Tank</a>");
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
  html += F("let _devEpoch=null; let _tickTimer=null;");
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
  html += F("if(press) press.textContent = (typeof st.pressure==='number' && st.pressure>0) ? st.pressure : '--';");

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
  html += F("    add('zoneName'+z); add('startHour'+z); add('startMin'+z); add('startHour2'+z); add('startMin2'+z); add('durationMin'+z); add('durationSec'+z);");
  html += F("    add('enableStartTime2'+z);");
  html += F("    for(let d=0; d<7; d++) add('day'+z+'_'+d);");
  html += F("  }");
  html += F("  try{ await fetch('/submit',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:fd.toString()}); location.reload(); }catch(e){ console.error(e); }");
  html += F("} ");
  html += F("document.getElementById('btn-save-all')?.addEventListener('click', saveAll);");

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
  html += F(".card h3{margin:0 0 12px 0;font-size:1.1em;font-weight:800;letter-spacing:.3px;color:#e8eef6}");
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

  // Help text row: let the text wrap nicely
  html += F(".helptext label{min-width:0}");
  html += F(".helptext small{max-width:520px;display:block}");

  // Make native select look like themed dropdown with a chevron
  html += F("select{appearance:none;-webkit-appearance:none;-moz-appearance:none;padding-right:32px;");
  html += F("background-image:linear-gradient(45deg,transparent 50%,#9aa6c2 50%),linear-gradient(135deg,#9aa6c2 50%,transparent 50%);");
  html += F("background-position:calc(100% - 18px) 50%,calc(100% - 13px) 50%;background-size:5px 5px,5px 5px;background-repeat:no-repeat;}");

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
  html += F("<div class='card'><h3>Weather</h3>");
  html += F("<div class='row'><label>API Key</label><input class='in-wide' type='text' name='apiKey' value='"); html += apiKey; html += F("'></div>");
  html += F("<div class='row'><label>City ID</label><input class='in-med' type='text' name='city' value='"); html += city; html += F("'><small>OpenWeatherMap city id</small></div>");
  html += F("</div>");

  // Zones
  html += F("<div class='card'><h3>Zones</h3>");
  html += F("<div class='row switchline'><label>Mode</label>");
  html += F("<label><input type='radio' name='zonesMode' value='4' "); html += (zonesCount==4?"checked":""); html += F("> 4 Zone + Mains/Tank</label>");
  html += F("<label><input type='radio' name='zonesMode' value='6' "); html += (zonesCount==6?"checked":""); html += F("> 6 Zone</label></div>");

  // Run mode
  html += F("<div class='row switchline'><label>Run Mode</label>");
  html += F("<label><input type='checkbox' name='runConcurrent' "); html += (runZonesConcurrent ? "checked" : "");
  html += F("> Run Zones Together</label><small>Unchecked = One at a time</small></div>");
  html += F("</div>");

  if (zonesCount == 4) {
    // Tank
    html += F("<div class='card'><h3>Tank & Water Source</h3>");
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

    html += F("<small>(4-zone Tank/Mains mode only)</small></div>");

    html += F("<div class='row'><label>Tank Low Threshold (%)</label>"
              "<input class='in-xs' type='number' min='0' max='100' name='tankThresh' value='");
    html += String(tankLowThresholdPct);
    html += F("'><small>Switch to mains if tank below this level</small></div>");

    html += F("<div class='row'><label>Tank Sensor GPIO</label><input class='in-xs' type='number' min='0' max='39' name='tankLevelPin' value='");
    html += String(tankLevelPin); html += F("'><small>ADC pin, e.g. 36 (GPIO36)</small></div>");
    html += F("</div>");
  }

  // Physical rain & forecast
  html += F("<div class='card'><h3>Rain Sources</h3>");
  html += F("<div class='row switchline'><label>Disable OWM Rain</label><input type='checkbox' name='rainForecastDisabled' ");
  html += (!rainDelayFromForecastEnabled ? "checked" : ""); html += F("><small>Checked = ignore OpenWeatherMap rain</small></div>");
  html += F("<div class='row switchline'><label>Enable Rain Sensor</label><input type='checkbox' name='rainSensorEnabled' "); html += (rainSensorEnabled?"checked":""); html += F("></div>");
  html += F("<div class='row'><label>Rain Sensor GPIO</label><input class='in-xs' type='number' min='0' max='39' name='rainSensorPin' value='"); html += String(rainSensorPin); html += F("'><small>e.g. 27</small></div>");
  html += F("<div class='row switchline'><label>Invert Sensor</label><input type='checkbox' name='rainSensorInvert' "); html += (rainSensorInvert?"checked":""); html += F("><small>Use if board is NO</small></div>");
  html += F("</div>");

  // Delays & Pause + thresholds
  html += F("<div class='card'><h3>Delays & Pause</h3>");
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
  html += F("<button class='btn' type='button' id='btn-resume'>Resume</button>");
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
  html += F("<div class='row'><label>Pause for (hours)</label><input class='in-sm' type='number' min='0' max='720' name='pauseHours' value='");
  time_t nowEp = time(nullptr);
  {
    uint32_t remain = (pauseUntilEpoch > nowEp && systemPaused) ? (pauseUntilEpoch - nowEp) : 0;
    html += String(remain/3600);
  }
  html += F("'><small>0 = until manually resumed</small></div>");
  html += F("</div>");

  html += F("</div>"); // end cols2
  html += F("</div>"); // end Delays card

  // Actions
  html += F("<div class='card'><h3>Actions</h3>");
  html += F("<div class='row' style='gap:8px;flex-wrap:wrap'>");
  html += F("<button class='btn' type='submit'>Save</button>");
  html += F("<button class='btn-alt' formaction='/' formmethod='GET'>Home</button>");
  html += F("<button class='btn-alt' type='button' onclick=\"fetch('/clear_cooldown',{method:'POST'})\">Clear Cooldown</button>");
  html += F("<button class='btn-alt' formaction='/configure' formmethod='POST' name='resumeNow' value='1'>Resume Now</button>");
  html += F("<button class='btn-alt' type='button' id='btn-toggle-backlight'>Toggle LCD</button>");
  html += F("</div></div>");

  // GPIO fallback pins
  html += F("<div class='card'><h3>GPIO Fallback (if I2C relays not found)</h3><div class='grid'>");
  for (uint8_t i=0;i<MAX_ZONES;i++){
    html += F("<div class='row'><label>Zone "); html += String(i+1);
    html += F(" Pin</label><input class='in-xs' type='number' min='0' max='39' name='zonePin"); html += String(i);
    html += F("' value='"); html += String(zonePins[i]); html += F("'></div>");
  }
  html += F("<div class='row'><label>Main Pin</label><input class='in-xs' type='number' min='0' max='39' name='mainsPin' value='");
  html += String(mainsPin); html += F("'><small>4-zone fallback</small></div>");
  html += F("<div class='row'><label>Tank Relay Pin</label><input class='in-xs' type='number' min='0' max='39' name='tankPin' value='");
  html += String(tankPin); html += F("'><small>4-zone fallback relay</small></div>");

  // NEW: GPIO active polarity
  html += F("<div class='row switchline'><label>GPIO Active Low</label>");
  html += F("<input type='checkbox' name='gpioActiveLow' ");
  html += (gpioActiveLow ? "checked" : "");
  html += F(">");
  html += F("<small>Checked = LOW = ON (active-low relay modules). Unchecked = HIGH = ON.</small></div>");

  html += F("</div></div>");

  // Manual buttons
  html += F("<div class='card'><h3>Manual Buttons</h3>");
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
  html += F("<div class='card'><h3>Timezone</h3>");

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
  html += F("<div class='card'><h3>MQTT (Home Assistant)</h3>");
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
  int raw=analogRead(tankLevelPin);
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
  html += F("<p>Level: <b>"); html += String(pct); html += F("%</b></p>");
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
    uint8_t z = s.toInt();
    zonesCount = (z == 6 ? 6 : 4);
  }

  if ((s = _safeReadLine(f)).length()) rainSensorEnabled = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) rainSensorPin     = s.toInt();
  if ((s = _safeReadLine(f)).length()) rainSensorInvert  = (s.toInt() == 1);
  if ((s = _safeReadLine(f)).length()) {
    int th = s.toInt();
    if (th >= 0 && th <= 100) tankLowThresholdPct = th;
  }

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    if ((s = _safeReadLine(f)).length()) zonePins[i] = s.toInt();
  }
  if ((s = _safeReadLine(f)).length()) mainsPin = s.toInt();
  if ((s = _safeReadLine(f)).length()) tankPin  = s.toInt();
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
      if (p >= 0 && p <= 39) tankLevelPin = p;
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

    int idx=0;
    auto next=[&](int& outIdx){
      int nidx=line.indexOf(',',idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim(); outIdx=nidx;
      int v = sv.toInt(); idx=nidx+1; return v;
    };

    int tmp;
    startHour[i]   = next(tmp);
    startMin[i]    = next(tmp);
    startHour2[i]  = next(tmp);
    startMin2[i]   = next(tmp);
    durationMin[i] = next(tmp);
    durationSec[i] = next(tmp);
    enableStartTime2[i] = (next(tmp)==1);

    for (int d=0; d<7; d++) {
      int nidx=line.indexOf(',',idx); if (nidx<0) nidx=line.length();
      String sv=line.substring(idx,nidx); sv.trim();
      days[i][d] = (sv.toInt()==1);
      idx = (nidx<(int)line.length()) ? nidx+1 : nidx;
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

  // Weather
  if (server.hasArg("apiKey")) apiKey = server.arg("apiKey");
  if (server.hasArg("city"))   city   = server.arg("city");

  // Zones mode (4 + tank/mains OR 6-zone)
  if (server.hasArg("zonesMode")) {
    String zm = server.arg("zonesMode");
    zonesCount = (zm == "6") ? 6 : 4;
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

  // GPIO fallback pins
  for (int i = 0; i < MAX_ZONES; i++) {
    String key = "zonePin" + String(i);
    if (server.hasArg(key)) {
      int p = server.arg(key).toInt();
      if (p >= 0 && p <= 39) zonePins[i] = p;
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
    if (p >= 0 && p <= 39) tankLevelPin = p;
  }
  if (server.hasArg("manualSelectPin")) {
    int p = server.arg("manualSelectPin").toInt();
    if ((p >= -1 && p <= 39)) manualSelectPin = p;
  }
  if (server.hasArg("manualStartPin")) {
    int p = server.arg("manualStartPin").toInt();
    if ((p >= -1 && p <= 39)) manualStartPin = p;
  }

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
}

void handleClearEvents() {
  HttpScope _scope;
  if (LittleFS.exists("/events.csv")) {
    LittleFS.remove("/events.csv");
  }
  server.sendHeader("Location", "/events", true);
  server.send(302, "text/plain", "");
}
