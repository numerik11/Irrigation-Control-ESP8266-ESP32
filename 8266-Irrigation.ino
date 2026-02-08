#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <time.h>
#include <math.h>

// ===== Simplified WiFi Irrigation (ESP8266 + 16x2 I2C LCD) =====
// - 4..6 zones
// - Sequential run only (one zone at a time)
// - Two start times per zone
// - LittleFS config + schedule persistence

static const uint8_t MAX_ZONES = 6;

LiquidCrystal_I2C lcd(0x27, 16, 2);
WiFiManager wifiManager;
ESP8266WebServer server(80);

// Defaults are safe for D1 mini while keeping I2C on D1/D2.
int zonePins[MAX_ZONES] = {D0, D5, D6, D7, -1, -1};
uint8_t zonesCount = 4; // allowed: 1..6

String zoneNames[MAX_ZONES] = {
  "Zone 1", "Zone 2", "Zone 3", "Zone 4", "Zone 5", "Zone 6"
};

// Timezone offset in hours from UTC.
float tzOffsetHours = 10.0f;

// Weather control (Open-Meteo current endpoint).
bool weatherEnabled = false;
float weatherLat = 0.0f;
float weatherLon = 0.0f;
bool windDelayEnabled = false;
float windSpeedThreshold = 30.0f; // km/h
bool rainCancelEnabled = false;

// Schedule
int  startHour[MAX_ZONES]    = {6, 6, 6, 6, 6, 6};
int  startMin[MAX_ZONES]     = {0, 10, 20, 30, 40, 50};
int  startHour2[MAX_ZONES]   = {0, 0, 0, 0, 0, 0};
int  startMin2[MAX_ZONES]    = {0, 0, 0, 0, 0, 0};
int  durationMin[MAX_ZONES]  = {10, 10, 10, 10, 10, 10};
int  duration2Min[MAX_ZONES] = {0, 0, 0, 0, 0, 0};
bool enableStart2[MAX_ZONES] = {false, false, false, false, false, false};

// day index: 0=Sun..6=Sat
bool days[MAX_ZONES][7] = {
  {true, true, true, true, true, true, true},
  {true, true, true, true, true, true, true},
  {true, true, true, true, true, true, true},
  {true, true, true, true, true, true, true},
  {true, true, true, true, true, true, true},
  {true, true, true, true, true, true, true}
};

// Runtime state
bool zoneActive[MAX_ZONES] = {false};
bool pendingStart[MAX_ZONES] = {false};
uint8_t pendingSlot[MAX_ZONES] = {1, 1, 1, 1, 1, 1};
uint8_t lastStartSlot[MAX_ZONES] = {1, 1, 1, 1, 1, 1};
uint32_t zoneStartMs[MAX_ZONES] = {0};
uint32_t zoneRunTotalSec[MAX_ZONES] = {0};
time_t lastStartMinuteEpoch[MAX_ZONES] = {0, 0, 0, 0, 0, 0};
time_t lastSchedulerMinuteEpoch = 0;

uint32_t lastLcdMs = 0;
uint32_t lastTickMs = 0;
uint32_t lastWeatherFetchMs = 0;
uint32_t lastWeatherOkMs = 0;

bool weatherValid = false;
float weatherTempC = 0.0f;
int weatherHumidity = -1;
float weatherWindKmh = 0.0f;
float weatherPrecipMm = 0.0f;
int weatherCode = -1;
String weatherError;

bool windDelayActive = false;
bool rainActive = false;

static void turnOffZone(int z);

static inline int clampInt(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static bool validZonePin(int pin) {
  if (pin < 0) return false;
  return pin <= 16;
}

static bool weatherCodeIsWet(int code) {
  if (code < 0) return false;
  if (code >= 51 && code <= 67) return true;
  if (code >= 71 && code <= 77) return true;
  if (code >= 80 && code <= 86) return true;
  if (code >= 95 && code <= 99) return true;
  return false;
}

static bool weatherRulesArmed() {
  return fabsf(weatherLat) > 0.01f || fabsf(weatherLon) > 0.01f;
}

static uint32_t durationForSlot(int z, int slot) {
  if (z < 0 || z >= (int)MAX_ZONES) return 0;
  int mins = (slot == 2) ? duration2Min[z] : durationMin[z];
  mins = clampInt(mins, 0, 600);
  return (uint32_t)mins * 60UL;
}

static time_t minuteEpoch(time_t t) {
  return t - (t % 60);
}

static bool anyZoneOn() {
  for (int i = 0; i < (int)zonesCount; i++) {
    if (zoneActive[i]) return true;
  }
  return false;
}

static void lcdShow(const String& line1, const String& line2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

static void initZonePins() {
  for (int i = 0; i < (int)MAX_ZONES; i++) {
    if (!validZonePin(zonePins[i])) continue;
    pinMode(zonePins[i], OUTPUT);
    digitalWrite(zonePins[i], LOW);
  }
}

static void allZonesOff() {
  for (int z = 0; z < (int)zonesCount; z++) {
    if (validZonePin(zonePins[z])) digitalWrite(zonePins[z], LOW);
    zoneActive[z] = false;
    zoneRunTotalSec[z] = 0;
  }
}

static void saveConfig() {
  File f = LittleFS.open("/config.txt", "w");
  if (!f) return;

  f.println(String(tzOffsetHours, 2));
  f.println((int)zonesCount);
  for (int i = 0; i < (int)MAX_ZONES; i++) f.println(zonePins[i]);
  for (int i = 0; i < (int)MAX_ZONES; i++) f.println(zoneNames[i]);
  f.println(weatherEnabled ? 1 : 0);
  f.println(String(weatherLat, 6));
  f.println(String(weatherLon, 6));
  f.println(windDelayEnabled ? 1 : 0);
  f.println(String(windSpeedThreshold, 2));
  f.println(rainCancelEnabled ? 1 : 0);

  f.close();
}

static void loadConfig() {
  if (!LittleFS.exists("/config.txt")) return;

  File f = LittleFS.open("/config.txt", "r");
  if (!f) return;

  auto readLine = [&](String& out) {
    if (!f.available()) {
      out = "";
      return;
    }
    out = f.readStringUntil('\n');
    out.trim();
  };

  String s;
  readLine(s);
  if (s.length()) tzOffsetHours = s.toFloat();

  readLine(s);
  if (s.length()) {
    int zc = s.toInt();
    zonesCount = (uint8_t)clampInt(zc, 1, 6);
  }

  for (int i = 0; i < (int)MAX_ZONES; i++) {
    readLine(s);
    if (s.length()) zonePins[i] = s.toInt();
  }

  for (int i = 0; i < (int)MAX_ZONES; i++) {
    readLine(s);
    if (s.length()) zoneNames[i] = s;
  }

  readLine(s);
  if (s.length()) weatherEnabled = (s.toInt() == 1);
  readLine(s);
  if (s.length()) weatherLat = s.toFloat();
  readLine(s);
  if (s.length()) weatherLon = s.toFloat();
  readLine(s);
  if (s.length()) windDelayEnabled = (s.toInt() == 1);
  readLine(s);
  if (s.length()) windSpeedThreshold = s.toFloat();
  readLine(s);
  if (s.length()) rainCancelEnabled = (s.toInt() == 1);

  if (weatherLat < -90.0f) weatherLat = -90.0f;
  if (weatherLat > 90.0f) weatherLat = 90.0f;
  if (weatherLon < -180.0f) weatherLon = -180.0f;
  if (weatherLon > 180.0f) weatherLon = 180.0f;
  if (windSpeedThreshold < 1.0f) windSpeedThreshold = 30.0f;
  if (windSpeedThreshold > 200.0f) windSpeedThreshold = 200.0f;

  f.close();
}

static void saveSchedule() {
  File f = LittleFS.open("/schedule.txt", "w");
  if (!f) return;

  for (int z = 0; z < (int)MAX_ZONES; z++) {
    f.print(startHour[z]); f.print(',');
    f.print(startMin[z]); f.print(',');
    f.print(startHour2[z]); f.print(',');
    f.print(startMin2[z]); f.print(',');
    f.print(durationMin[z]); f.print(',');
    f.print(duration2Min[z]); f.print(',');
    f.print(enableStart2[z] ? 1 : 0);
    for (int d = 0; d < 7; d++) {
      f.print(',');
      f.print(days[z][d] ? 1 : 0);
    }
    f.println();
  }

  f.close();
}

static void loadSchedule() {
  if (!LittleFS.exists("/schedule.txt")) return;

  File f = LittleFS.open("/schedule.txt", "r");
  if (!f) return;

  for (int z = 0; z < (int)MAX_ZONES && f.available(); z++) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;

    int tokens[16] = {0};
    int count = 0;
    int from = 0;
    while (from <= (int)line.length() && count < 16) {
      int comma = line.indexOf(',', from);
      if (comma < 0) comma = line.length();
      String part = line.substring(from, comma);
      part.trim();
      tokens[count++] = part.toInt();
      from = comma + 1;
    }

    auto tok = [&](int idx, int def) { return idx < count ? tokens[idx] : def; };

    startHour[z]   = clampInt(tok(0, startHour[z]), 0, 23);
    startMin[z]    = clampInt(tok(1, startMin[z]), 0, 59);
    startHour2[z]  = clampInt(tok(2, startHour2[z]), 0, 23);
    startMin2[z]   = clampInt(tok(3, startMin2[z]), 0, 59);
    durationMin[z] = clampInt(tok(4, durationMin[z]), 0, 600);
    duration2Min[z]= clampInt(tok(5, duration2Min[z]), 0, 600);
    enableStart2[z]= tok(6, enableStart2[z] ? 1 : 0) == 1;

    for (int d = 0; d < 7; d++) {
      days[z][d] = tok(7 + d, days[z][d] ? 1 : 0) == 1;
    }
  }

  f.close();
}

static void applyTimeConfig() {
  long offsetSec = (long)(tzOffsetHours * 3600.0f);
  configTime(offsetSec, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
}

static String fmtHHMM(int h, int m) {
  char b[6];
  snprintf(b, sizeof(b), "%02d:%02d", h, m);
  return String(b);
}

static bool fetchWeatherNow() {
  if (WiFi.status() != WL_CONNECTED) {
    weatherError = "wifi";
    weatherValid = false;
    return false;
  }
  if (weatherLat < -90.0f || weatherLat > 90.0f || weatherLon < -180.0f || weatherLon > 180.0f) {
    weatherError = "coords";
    weatherValid = false;
    return false;
  }

  BearSSL::WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = "https://api.open-meteo.com/v1/forecast?latitude=" + String(weatherLat, 6) +
               "&longitude=" + String(weatherLon, 6) +
               "&current=temperature_2m,relative_humidity_2m,precipitation,weather_code,wind_speed_10m&wind_speed_unit=kmh";
  if (!http.begin(client, url)) {
    weatherError = "http_begin";
    weatherValid = false;
    return false;
  }

  int rc = http.GET();
  if (rc != HTTP_CODE_OK) {
    weatherError = "http_" + String(rc);
    weatherValid = false;
    http.end();
    return false;
  }

  String body = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err) {
    weatherError = "json";
    weatherValid = false;
    return false;
  }

  JsonVariant cur = doc["current"];
  if (cur.isNull()) {
    weatherError = "no_current";
    weatherValid = false;
    return false;
  }

  weatherTempC = cur["temperature_2m"] | 0.0f;
  weatherHumidity = cur["relative_humidity_2m"] | -1;
  weatherPrecipMm = cur["precipitation"] | 0.0f;
  weatherCode = cur["weather_code"] | -1;
  weatherWindKmh = cur["wind_speed_10m"] | 0.0f;
  weatherValid = true;
  lastWeatherOkMs = millis();
  weatherError = "";
  return true;
}

static void updateWeatherCache(uint32_t nowMs) {
  if (!weatherEnabled) {
    weatherValid = false;
    weatherError = "";
    return;
  }

  const uint32_t intervalMs = 180000UL;
  bool due = (lastWeatherFetchMs == 0) || ((nowMs - lastWeatherFetchMs) >= intervalMs);
  if (!due) return;

  fetchWeatherNow();
  lastWeatherFetchMs = nowMs;
}

static void evaluateWeatherBlocks() {
  const uint32_t staleMs = 10UL * 60UL * 1000UL;
  bool stale = (lastWeatherOkMs == 0) || ((millis() - lastWeatherOkMs) > staleMs);

  if (!weatherEnabled || !weatherRulesArmed() || !weatherValid || stale) {
    windDelayActive = false;
    rainActive = false;
    return;
  }

  bool rainNow = rainCancelEnabled && (weatherPrecipMm > 0.05f || weatherCodeIsWet(weatherCode));
  bool windNow = windDelayEnabled && (weatherWindKmh >= windSpeedThreshold);

  windDelayActive = windNow;

  if (rainNow) {
    for (int z = 0; z < (int)zonesCount; z++) {
      if (zoneActive[z]) turnOffZone(z);
      pendingStart[z] = false;
    }
  }
  rainActive = rainNow;
}

static bool turnOnZone(int z, uint8_t slot) {
  if (z < 0 || z >= (int)zonesCount) return false;
  if (!validZonePin(zonePins[z])) {
    Serial.printf("[zone %d] start blocked: invalid pin %d\r\n", z + 1, zonePins[z]);
    return false;
  }
  if (rainActive) {
    Serial.printf("[zone %d] start blocked: rain rule\r\n", z + 1);
    return false;
  }
  if (windDelayActive) {
    pendingStart[z] = true;
    pendingSlot[z] = slot;
    Serial.printf("[zone %d] queued: wind delay\r\n", z + 1);
    return true;
  }

  if (anyZoneOn()) {
    pendingStart[z] = true;
    pendingSlot[z] = slot;
    Serial.printf("[zone %d] queued: another zone running\r\n", z + 1);
    return true;
  }

  uint32_t dur = durationForSlot(z, slot);
  if (dur == 0) {
    Serial.printf("[zone %d] start blocked: duration is zero (slot %d)\r\n", z + 1, slot);
    return false;
  }

  digitalWrite(zonePins[z], HIGH);
  zoneActive[z] = true;
  zoneStartMs[z] = millis();
  zoneRunTotalSec[z] = dur;
  lastStartSlot[z] = slot;
  pendingStart[z] = false;

  lcdShow(zoneNames[z] + " ON", "Run " + String(dur / 60) + "m");
  Serial.printf("[zone %d] ON slot %d, duration %lus\r\n", z + 1, slot, (unsigned long)dur);
  return true;
}

static void turnOffZone(int z) {
  if (z < 0 || z >= (int)zonesCount) return;
  if (validZonePin(zonePins[z])) digitalWrite(zonePins[z], LOW);

  zoneActive[z] = false;
  zoneRunTotalSec[z] = 0;

  lcdShow(zoneNames[z] + " OFF", "");
}

static void drainQueue() {
  if (anyZoneOn() || rainActive || windDelayActive) return;
  for (int z = 0; z < (int)zonesCount; z++) {
    if (pendingStart[z]) {
      pendingStart[z] = false;
      turnOnZone(z, pendingSlot[z]);
      return;
    }
  }
}

static bool hasDurationCompleted(int z) {
  if (!zoneActive[z]) return false;
  uint32_t elapsed = (millis() - zoneStartMs[z]) / 1000UL;
  return elapsed >= zoneRunTotalSec[z];
}

static void processDueStartsAtMinute(time_t minuteMark) {
  tm t{};
  localtime_r(&minuteMark, &t);

  for (int z = 0; z < (int)zonesCount; z++) {
    if (lastStartMinuteEpoch[z] == minuteMark) continue;
    if (!days[z][t.tm_wday]) continue;

    bool match1 = (t.tm_hour == startHour[z] && t.tm_min == startMin[z]);
    bool match2 = enableStart2[z] && (t.tm_hour == startHour2[z] && t.tm_min == startMin2[z]);
    if (!match1 && !match2) continue;

    uint8_t slot = match2 ? 2 : 1;
    if (durationForSlot(z, slot) == 0) continue;

    if (turnOnZone(z, slot)) {
      lastStartMinuteEpoch[z] = minuteMark;
    }
  }
}

static void catchUpDueStartsNow() {
  time_t now = time(nullptr);
  if (now <= 0) return;
  processDueStartsAtMinute(minuteEpoch(now));
}

static void lcdHome() {
  time_t now = time(nullptr);
  tm t{};
  localtime_r(&now, &t);

  int running = -1;
  for (int z = 0; z < (int)zonesCount; z++) {
    if (zoneActive[z]) {
      running = z;
      break;
    }
  }

  if (running >= 0) {
    uint32_t elapsed = (millis() - zoneStartMs[running]) / 1000UL;
    uint32_t rem = zoneRunTotalSec[running] > elapsed ? zoneRunTotalSec[running] - elapsed : 0;
    char l1[17];
    char l2[17];
    snprintf(l1, sizeof(l1), "Z%d %-12s", running + 1, "Running");
    snprintf(l2, sizeof(l2), "R %02lu:%02lu", rem / 60UL, rem % 60UL);
    lcdShow(String(l1), String(l2));
    return;
  }

  int queued = 0;
  for (int z = 0; z < (int)zonesCount; z++) if (pendingStart[z]) queued++;

  char l1[17];
  String l2;
  snprintf(l1, sizeof(l1), "%02d:%02d:%02d WiFi", t.tm_hour, t.tm_min, t.tm_sec);
  l2 = "Z:" + String((int)zonesCount) + " Q:" + String(queued);
  lcdShow(String(l1), l2);
}

static void handleValveOn() {
  if (!server.hasArg("z")) {
    server.send(400, "text/plain", "missing z");
    return;
  }
  int z = server.arg("z").toInt();
  if (z < 0 || z >= (int)zonesCount) {
    server.send(400, "text/plain", "bad z");
    return;
  }
  turnOnZone(z, 1);
  server.send(200, "text/plain", "OK");
}

static void handleValveOff() {
  if (!server.hasArg("z")) {
    server.send(400, "text/plain", "missing z");
    return;
  }
  int z = server.arg("z").toInt();
  if (z < 0 || z >= (int)zonesCount) {
    server.send(400, "text/plain", "bad z");
    return;
  }
  turnOffZone(z);
  server.send(200, "text/plain", "OK");
}

static void handleSubmit() {
  for (int z = 0; z < (int)zonesCount; z++) {
    String key;

    key = "name" + String(z);
    if (server.hasArg(key)) {
      zoneNames[z] = server.arg(key);
      zoneNames[z].trim();
      if (!zoneNames[z].length()) zoneNames[z] = "Zone " + String(z + 1);
    }

    key = "sh" + String(z);
    if (server.hasArg(key)) startHour[z] = clampInt(server.arg(key).toInt(), 0, 23);

    key = "sm" + String(z);
    if (server.hasArg(key)) startMin[z] = clampInt(server.arg(key).toInt(), 0, 59);

    key = "sh2_" + String(z);
    if (server.hasArg(key)) startHour2[z] = clampInt(server.arg(key).toInt(), 0, 23);

    key = "sm2_" + String(z);
    if (server.hasArg(key)) startMin2[z] = clampInt(server.arg(key).toInt(), 0, 59);

    key = "dur" + String(z);
    if (server.hasArg(key)) durationMin[z] = clampInt(server.arg(key).toInt(), 0, 600);

    key = "dur2_" + String(z);
    if (server.hasArg(key)) duration2Min[z] = clampInt(server.arg(key).toInt(), 0, 600);

    key = "en2_" + String(z);
    enableStart2[z] = server.hasArg(key);

    for (int d = 0; d < 7; d++) {
      key = "d" + String(z) + "_" + String(d);
      days[z][d] = server.hasArg(key);
    }
  }

  saveSchedule();
  saveConfig();
  catchUpDueStartsNow();
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

static void handleConfigure() {
  if (server.hasArg("zonesCount")) {
    zonesCount = (uint8_t)clampInt(server.arg("zonesCount").toInt(), 1, 6);
  }

  if (server.hasArg("tz")) {
    tzOffsetHours = server.arg("tz").toFloat();
    if (tzOffsetHours < -12.0f) tzOffsetHours = -12.0f;
    if (tzOffsetHours > 14.0f) tzOffsetHours = 14.0f;
  }

  weatherEnabled = server.hasArg("wx_en");
  if (server.hasArg("wx_lat")) {
    weatherLat = server.arg("wx_lat").toFloat();
    if (weatherLat < -90.0f) weatherLat = -90.0f;
    if (weatherLat > 90.0f) weatherLat = 90.0f;
  }
  if (server.hasArg("wx_lon")) {
    weatherLon = server.arg("wx_lon").toFloat();
    if (weatherLon < -180.0f) weatherLon = -180.0f;
    if (weatherLon > 180.0f) weatherLon = 180.0f;
  }
  windDelayEnabled = server.hasArg("wind_en");
  if (server.hasArg("wind_thr")) {
    windSpeedThreshold = server.arg("wind_thr").toFloat();
    if (windSpeedThreshold < 1.0f) windSpeedThreshold = 30.0f;
    if (windSpeedThreshold > 200.0f) windSpeedThreshold = 200.0f;
  }
  rainCancelEnabled = server.hasArg("rain_cancel");

  for (int z = 0; z < (int)MAX_ZONES; z++) {
    String key = "pin" + String(z);
    if (server.hasArg(key)) {
      int p = server.arg(key).toInt();
      zonePins[z] = (p >= -1 && p <= 16) ? p : -1;
    }

    key = "zname" + String(z);
    if (server.hasArg(key)) {
      zoneNames[z] = server.arg(key);
      zoneNames[z].trim();
      if (!zoneNames[z].length()) zoneNames[z] = "Zone " + String(z + 1);
    }
  }

  saveConfig();
  initZonePins();
  applyTimeConfig();
  lastWeatherFetchMs = 0;
  updateWeatherCache(millis());
  evaluateWeatherBlocks();

  server.sendHeader("Location", "/setup", true);
  server.send(302, "text/plain", "");
}

static void handleStatus() {
  String out;
  out.reserve(1200);

  time_t now = time(nullptr);
  out += F("{\"deviceEpoch\":");
  out += String((unsigned long)now);
  out += F(",\"weather\":{");
  out += F("\"enabled\":");
  out += (weatherEnabled ? F("true") : F("false"));
  out += F(",\"valid\":");
  out += (weatherValid ? F("true") : F("false"));
  out += F(",\"tempC\":");
  out += String(weatherTempC, 1);
  out += F(",\"humidity\":");
  out += String(weatherHumidity);
  out += F(",\"windKmh\":");
  out += String(weatherWindKmh, 1);
  out += F(",\"precipMm\":");
  out += String(weatherPrecipMm, 2);
  out += F(",\"code\":");
  out += String(weatherCode);
  out += F(",\"rainActive\":");
  out += (rainActive ? F("true") : F("false"));
  out += F(",\"windDelayActive\":");
  out += (windDelayActive ? F("true") : F("false"));
  out += F(",\"blockReason\":\"");
  if (rainActive) out += F("rain");
  else if (windDelayActive) out += F("wind");
  else out += F("");
  out += F("\"");
  out += F(",\"error\":\"");
  out += weatherError;
  out += F("\"}");
  out += F(",\"zones\":[");

  for (int z = 0; z < (int)zonesCount; z++) {
    if (z) out += ',';

    uint32_t rem = 0;
    if (zoneActive[z]) {
      uint32_t elapsed = (millis() - zoneStartMs[z]) / 1000UL;
      rem = (zoneRunTotalSec[z] > elapsed) ? (zoneRunTotalSec[z] - elapsed) : 0;
    }

    out += F("{\"active\":");
    out += (zoneActive[z] ? F("true") : F("false"));
    out += F(",\"queued\":");
    out += (pendingStart[z] ? F("true") : F("false"));
    out += F(",\"remaining\":");
    out += String(rem);
    out += F(",\"pin\":");
    out += String(zonePins[z]);
    out += F(",\"pinValid\":");
    out += (validZonePin(zonePins[z]) ? F("true") : F("false"));
    out += F("}");
  }

  out += F("]}");
  server.send(200, "application/json", out);
}

static void handleRoot() {
  time_t now = time(nullptr);
  tm t{};
  localtime_r(&now, &t);

  String html;
  html.reserve(26000);
  html += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Irrigation</title><style>");
  html += F(":root{--bg:#f3f7f4;--bg2:#edf6ff;--card:rgba(255,255,255,.82);--line:#c8d9d3;--text:#122128;--muted:#526570;--pri:#008c72;--pri2:#00695a;--off:#596a78;--ring:rgba(0,140,114,.2)}");
  html += F("html[data-theme='dark']{--bg:#0c1720;--bg2:#112431;--card:rgba(18,33,46,.8);--line:#2b4254;--text:#eaf3f8;--muted:#9fb5c3;--pri:#1aa988;--pri2:#12836a;--off:#61798b;--ring:rgba(26,169,136,.25)}");
  html += F("*{box-sizing:border-box}body{font-family:'Avenir Next','Trebuchet MS','Verdana',sans-serif;background:linear-gradient(160deg,var(--bg),var(--bg2));color:var(--text);margin:0;padding:20px;min-height:100vh;position:relative;overflow-x:hidden}");
  html += F("body:before,body:after{content:'';position:fixed;pointer-events:none;z-index:-1;border-radius:50%}body:before{width:42vw;height:42vw;left:-12vw;top:-14vw;background:radial-gradient(circle at center,rgba(0,140,114,.22),rgba(0,140,114,0) 70%)}");
  html += F("body:after{width:38vw;height:38vw;right:-10vw;bottom:-14vw;background:radial-gradient(circle at center,rgba(21,122,191,.18),rgba(21,122,191,0) 70%)}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(12px)}to{opacity:1;transform:translateY(0)}}");
  html += F(".wrap{max-width:1320px;margin:0 auto;animation:rise .35s ease-out}.head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:16px;flex-wrap:wrap}");
  html += F(".title{font-size:1.55rem;font-weight:900;letter-spacing:.25px}.sub{font-size:.88rem;color:var(--muted)}");
  html += F(".head-actions{display:flex;gap:8px;align-items:center}.btn-link,.btn-ghost{display:inline-flex;align-items:center;justify-content:center;padding:10px 14px;border-radius:12px;text-decoration:none;font-weight:800;transition:.18s transform,.18s box-shadow,.18s opacity}");
  html += F(".btn-link{background:linear-gradient(180deg,var(--pri),var(--pri2));color:#fff;box-shadow:0 8px 18px rgba(0,77,66,.25)}.btn-link:hover,.btn-ghost:hover,button:hover{transform:translateY(-1px)}");
  html += F(".btn-ghost{border:1px solid var(--line);background:var(--card);color:var(--text);cursor:pointer;backdrop-filter:blur(8px)}");
  html += F(".card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:16px;margin-bottom:15px;box-shadow:0 10px 30px rgba(12,35,52,.1);backdrop-filter:blur(10px);animation:rise .42s ease-out}");
  html += F(".pills{display:flex;flex-wrap:wrap;gap:8px}.pill{background:rgba(255,255,255,.65);border:1px solid var(--line);border-radius:999px;padding:7px 11px;font-size:.84rem;font-weight:650}html[data-theme='dark'] .pill{background:rgba(11,25,36,.66)}");
  html += F("h2{margin:0 0 12px 0;font-size:1.1rem;letter-spacing:.15px}.zones{display:grid;grid-template-columns:1fr;gap:12px}@media(min-width:960px){.zones{grid-template-columns:1fr 1fr}}");
  html += F(".zone{border:1px solid var(--line);border-radius:14px;padding:13px;background:rgba(255,255,255,.66);box-shadow:inset 0 1px 0 rgba(255,255,255,.45);animation:rise .5s ease-out}html[data-theme='dark'] .zone{background:rgba(15,31,44,.7)}");
  html += F(".zones .zone:nth-child(2n){animation-delay:.04s}.zhead{display:flex;justify-content:space-between;align-items:center;gap:8px;margin-bottom:8px}.ztitle{font-weight:850}.tag{font-size:.77rem;color:var(--muted)}");
  html += F(".row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}.row label{font-size:.82rem;color:var(--muted);min-width:72px}");
  html += F("input[type='number'],input[type='text']{padding:9px 10px;border:1px solid #b8cec6;border-radius:10px;background:#fff;color:var(--text);outline:0;transition:border-color .16s,box-shadow .16s}input:focus{border-color:var(--pri);box-shadow:0 0 0 3px var(--ring)}");
  html += F("html[data-theme='dark'] input[type='number'],html[data-theme='dark'] input[type='text']{background:#0f1f2d;border-color:#365163;color:var(--text)}");
  html += F("input[type='text']{min-width:170px}input[type='number']{width:76px}.dur{width:86px}input[type='checkbox']{accent-color:var(--pri)}");
  html += F(".days{display:grid;grid-template-columns:repeat(7,minmax(0,1fr));gap:6px;min-width:300px}.day{border:1px solid #bbd1c9;background:#fff;border-radius:9px;padding:5px 6px;font-size:.79rem;white-space:nowrap;text-align:center}");
  html += F("html[data-theme='dark'] .day{background:#0e1f2d;border-color:#345063}");
  html += F("button{padding:9px 12px;border:0;border-radius:11px;background:linear-gradient(180deg,var(--pri),var(--pri2));color:#fff;cursor:pointer;font-weight:800;transition:.18s transform,.18s box-shadow,.18s opacity}button:disabled{opacity:.55;cursor:not-allowed;transform:none}");
  html += F(".btn-off{background:linear-gradient(180deg,#6b7b89,#556675)}.actions{display:flex;flex-wrap:wrap;gap:8px;align-items:center}.state{font-size:.82rem;color:var(--muted)}");
  html += F(".savebar{position:sticky;bottom:0;background:rgba(243,247,244,.84);backdrop-filter:blur(7px);padding-top:8px;border-radius:12px}html[data-theme='dark'] .savebar{background:rgba(12,23,32,.82)}.savebtn{padding:11px 16px;font-size:.95rem}");
  html += F("@media(max-width:640px){body{padding:14px}.title{font-size:1.3rem}.head-actions{width:100%}.head-actions .btn-link,.head-actions .btn-ghost{flex:1}.row label{min-width:62px}.days{min-width:0}}");
  html += F("</style></head><body>");

  html += F("<div class='wrap'>");
  html += F("<div class='head'><div><div class='title'>WiFi Irrigation</div><div class='sub'>ESP8266 scheduler dashboard</div></div>");
  html += F("<div class='head-actions'><button class='btn-ghost' type='button' id='themeToggle'>Theme</button><a class='btn-link' href='/setup'>Setup</a></div></div>");

  html += F("<div class='card'><div class='pills'>");
  html += F("<div class='pill'><b>Time</b> <span id='clockVal'>");
  html += fmtHHMM(t.tm_hour, t.tm_min);
  html += F("</span>");
  html += F("</div>");
  html += F("<div class='pill'><b>WiFi</b> ");
  html += (WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  html += F("</div>");
  html += F("<div class='pill'><b>IP</b> ");
  html += WiFi.localIP().toString();
  html += F("</div>");
  html += F("<div class='pill'><b>Host</b> 8266Irrigation.local</div>");
  html += F("<div class='pill'><b>Weather</b> <span id='wxState'>");
  if (!weatherEnabled) html += F("Off");
  else if (!weatherValid) html += F("Updating...");
  else html += F("Live");
  html += F("</span></div>");
  html += F("<div class='pill'><b>Temp</b> <span id='wxTemp'>");
  html += String(weatherTempC, 1);
  html += F("C</span></div>");
  html += F("<div class='pill'><b>Wind</b> <span id='wxWind'>");
  html += String(weatherWindKmh, 1);
  html += F(" km/h</span></div>");
  html += F("<div class='pill'><b>Rain</b> <span id='wxRain'>");
  html += String(weatherPrecipMm, 2);
  html += F(" mm</span></div>");
  html += F("<div class='pill'><b>Weather Lock</b> <span id='wxLock'>");
  if (rainActive) html += F("Rain");
  else if (windDelayActive) html += F("Wind delay");
  else html += F("None");
  html += F("</span></div>");
  html += F("</div></div>");

  html += F("<form action='/submit' method='POST'><div class='card'><h2>Schedule</h2><div class='zones'>");
  for (int z = 0; z < (int)zonesCount; z++) {
    html += F("<div class='zone'><div class='zhead'><div class='ztitle'>");
    html += zoneNames[z];
    html += F("</div><div class='tag'>Pin ");
    html += String(zonePins[z]);
    html += F("</div></div>");

    html += F("<div class='row'><label>Name</label><input type='text' name='name"); html += String(z); html += F("' value='");
    html += zoneNames[z]; html += F("'></div>");

    html += F("<div class='row'><label>Start 1</label><input type='number' min='0' max='23' name='sh"); html += String(z); html += F("' value='"); html += String(startHour[z]); html += F("'>");
    html += F("<input type='number' min='0' max='59' name='sm"); html += String(z); html += F("' value='"); html += String(startMin[z]); html += F("'>");
    html += F("<label>Dur</label><input class='dur' type='number' min='0' max='600' name='dur"); html += String(z); html += F("' value='"); html += String(durationMin[z]); html += F("'></div>");

    html += F("<div class='row'><label><input type='checkbox' name='en2_"); html += String(z); html += F("' "); html += (enableStart2[z] ? "checked" : ""); html += F("> Start 2</label>");
    html += F("<input type='number' min='0' max='23' name='sh2_"); html += String(z); html += F("' value='"); html += String(startHour2[z]); html += F("'>");
    html += F("<input type='number' min='0' max='59' name='sm2_"); html += String(z); html += F("' value='"); html += String(startMin2[z]); html += F("'>");
    html += F("<label>Dur</label><input class='dur' type='number' min='0' max='600' name='dur2_"); html += String(z); html += F("' value='"); html += String(duration2Min[z]); html += F("'></div>");

    html += F("<div class='row'><label>Days</label><div class='days'>");
    const char* dn[7] = {"S", "M", "T", "W", "T", "F", "S"};
    for (int d = 0; d < 7; d++) {
      html += F("<label class='day'><input type='checkbox' name='d");
      html += String(z);
      html += F("_");
      html += String(d);
      html += F("' ");
      html += (days[z][d] ? "checked" : "");
      html += F("> ");
      html += dn[d];
      html += F("</label>");
    }
    html += F("</div></div>");

    html += F("<div class='actions'>");
    html += F("<button id='on-"); html += String(z); html += F("' type='button' onclick=\"zoneCmd("); html += String(z); html += F(",1)\">On</button>");
    html += F("<button id='off-"); html += String(z); html += F("' class='btn-off' type='button' onclick=\"zoneCmd("); html += String(z); html += F(",0)\">Off</button>");
    html += F("<span id='state-"); html += String(z); html += F("' class='state'>");
    if (zoneActive[z]) {
      uint32_t elapsed = (millis() - zoneStartMs[z]) / 1000UL;
      uint32_t rem = zoneRunTotalSec[z] > elapsed ? zoneRunTotalSec[z] - elapsed : 0;
      html += F("<b>Running</b> ");
      html += String(rem / 60);
      html += F("m ");
      html += String(rem % 60);
      html += F("s");
    } else if (pendingStart[z]) {
      html += F("<b>Queued</b>");
    } else if (!validZonePin(zonePins[z])) {
      html += F("<b>Blocked:</b> Pin");
    } else {
      html += F("Idle");
    }
    html += F("</span></div></div>");
  }
  html += F("</div><div class='savebar'><button class='savebtn' type='submit'>Save Schedule</button></div></div></form>");

  html += F("<script>");
  html += F("(function(){const k='ui_theme';let r=localStorage.getItem(k);if(!r){r=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';}document.documentElement.setAttribute('data-theme',r);");
  html += F("const b=document.getElementById('themeToggle');if(b){b.addEventListener('click',()=>{const c=document.documentElement.getAttribute('data-theme')==='dark'?'dark':'light';");
  html += F("const n=(c==='dark')?'light':'dark';document.documentElement.setAttribute('data-theme',n);localStorage.setItem(k,n);});}})();");
  html += F("let devEpoch="); html += String((unsigned long)now); html += F(";");
  html += F("const ZC="); html += String((unsigned)zonesCount); html += F(";");
  html += F("function pad(n){return n<10?'0'+n:n;}");
  html += F("function fmt(sec){sec=Math.max(0,sec|0);const m=(sec/60)|0;const s=sec%60;return m+'m '+pad(s)+'s';}");
  html += F("function drawClock(){const d=new Date(devEpoch*1000);const t=pad(d.getHours())+':'+pad(d.getMinutes())+':'+pad(d.getSeconds());const el=document.getElementById('clockVal');if(el)el.textContent=t;}");
  html += F("setInterval(()=>{devEpoch++;drawClock();},1000);drawClock();");
  html += F("async function refreshStatus(){try{const r=await fetch('/status');if(!r.ok)return;const st=await r.json();if(typeof st.deviceEpoch==='number')devEpoch=st.deviceEpoch;");
  html += F("if(st.weather){const w=st.weather||{};const ws=document.getElementById('wxState');const wt=document.getElementById('wxTemp');const ww=document.getElementById('wxWind');const wr=document.getElementById('wxRain');const wl=document.getElementById('wxLock');");
  html += F("if(ws){if(!w.enabled)ws.textContent='Off';else if(!w.valid)ws.textContent='Updating...';else ws.textContent='Live';}");
  html += F("if(wt&&typeof w.tempC==='number')wt.textContent=(Math.round(w.tempC*10)/10).toFixed(1)+'C';");
  html += F("if(ww&&typeof w.windKmh==='number')ww.textContent=(Math.round(w.windKmh*10)/10).toFixed(1)+' km/h';");
  html += F("if(wr&&typeof w.precipMm==='number')wr.textContent=(Math.round(w.precipMm*100)/100).toFixed(2)+' mm';");
  html += F("if(wl){if(w.rainActive)wl.textContent='Rain';else if(w.windDelayActive)wl.textContent='Wind delay';else wl.textContent='None';}}");
  html += F("if(Array.isArray(st.zones)){for(let z=0;z<Math.min(ZC,st.zones.length);z++){const it=st.zones[z]||{};const se=document.getElementById('state-'+z);const on=document.getElementById('on-'+z);const off=document.getElementById('off-'+z);");
  html += F("if(se){if(it.active){se.innerHTML='<b>Running</b> '+fmt(it.remaining||0);}else if(it.queued){se.innerHTML='<b>Queued</b>';}else if(it.pinValid===false){se.innerHTML='<b>Blocked:</b> Pin';}else if(st.weather&&st.weather.blockReason==='rain'){se.innerHTML='<b>Blocked:</b> Rain';}else if(st.weather&&st.weather.blockReason==='wind'){se.innerHTML='<b>Blocked:</b> Wind';}else{se.textContent='Idle';}}");
  html += F("if(on)on.disabled=!!it.active;if(off)off.disabled=!it.active;}}}catch(e){}}");
  html += F("async function zoneCmd(z,on){try{await fetch('/valve/'+(on?'on':'off')+'?z='+z,{method:'POST'});}catch(e){}setTimeout(refreshStatus,120);}"); 
  html += F("setInterval(refreshStatus,1000);refreshStatus();");
  html += F("</script>");

  html += F("</div></body></html>");
  server.send(200, "text/html", html);
}

static void handleSetupPage() {
  String html;
  html.reserve(15000);
  html += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>Setup</title><style>");
  html += F(":root{--bg:#f3f7f4;--bg2:#edf6ff;--card:rgba(255,255,255,.82);--line:#c8d9d3;--text:#122128;--muted:#526570;--pri:#008c72;--pri2:#00695a;--ring:rgba(0,140,114,.2)}");
  html += F("html[data-theme='dark']{--bg:#0c1720;--bg2:#112431;--card:rgba(18,33,46,.8);--line:#2b4254;--text:#eaf3f8;--muted:#9fb5c3;--pri:#1aa988;--pri2:#12836a;--ring:rgba(26,169,136,.25)}");
  html += F("*{box-sizing:border-box}body{font-family:'Avenir Next','Trebuchet MS','Verdana',sans-serif;background:linear-gradient(160deg,var(--bg),var(--bg2));color:var(--text);margin:0;padding:20px;min-height:100vh;position:relative;overflow-x:hidden}");
  html += F("body:before,body:after{content:'';position:fixed;pointer-events:none;z-index:-1;border-radius:50%}body:before{width:42vw;height:42vw;left:-12vw;top:-14vw;background:radial-gradient(circle at center,rgba(0,140,114,.22),rgba(0,140,114,0) 70%)}");
  html += F("body:after{width:38vw;height:38vw;right:-10vw;bottom:-14vw;background:radial-gradient(circle at center,rgba(21,122,191,.18),rgba(21,122,191,0) 70%)}");
  html += F("@keyframes rise{from{opacity:0;transform:translateY(12px)}to{opacity:1;transform:translateY(0)}}");
  html += F(".wrap{max-width:1080px;margin:0 auto;animation:rise .35s ease-out}.head{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:16px;flex-wrap:wrap}");
  html += F(".title{font-size:1.55rem;font-weight:900;letter-spacing:.25px}.sub{font-size:.88rem;color:var(--muted)}");
  html += F(".head-actions{display:flex;gap:8px;align-items:center}.btn-link,.btn-ghost{display:inline-flex;align-items:center;justify-content:center;padding:10px 14px;border-radius:12px;text-decoration:none;font-weight:800;transition:.18s transform,.18s box-shadow,.18s opacity}");
  html += F(".btn-link{background:linear-gradient(180deg,var(--pri),var(--pri2));color:#fff;box-shadow:0 8px 18px rgba(0,77,66,.25)}.btn-ghost{border:1px solid var(--line);background:var(--card);color:var(--text);cursor:pointer;backdrop-filter:blur(8px)}");
  html += F(".btn-link:hover,.btn-ghost:hover,.btn:hover,.link:hover{transform:translateY(-1px)}");
  html += F(".card{background:var(--card);border:1px solid var(--line);border-radius:18px;padding:16px;box-shadow:0 10px 30px rgba(12,35,52,.1);backdrop-filter:blur(10px);animation:rise .42s ease-out}");
  html += F(".pills{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}.pill{background:rgba(255,255,255,.65);border:1px solid var(--line);border-radius:999px;padding:7px 11px;font-size:.84rem;font-weight:650}html[data-theme='dark'] .pill{background:rgba(11,25,36,.66)}");
  html += F("h2{margin:0 0 12px 0;letter-spacing:.15px}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}.row label{min-width:140px;color:var(--muted);font-size:.88rem}");
  html += F("input,select{padding:9px 10px;border:1px solid #b8cec6;border-radius:10px;background:#fff;color:var(--text);outline:0;transition:border-color .16s,box-shadow .16s}input:focus,select:focus{border-color:var(--pri);box-shadow:0 0 0 3px var(--ring)}");
  html += F("html[data-theme='dark'] input,html[data-theme='dark'] select{background:#0f1f2d;border-color:#365163;color:var(--text)}input[type='checkbox']{accent-color:var(--pri)}");
  html += F("input[type='number']{width:95px}input[type='text']{min-width:180px}.help{font-size:.84rem;color:var(--muted)}");
  html += F(".zonegrid{display:grid;grid-template-columns:1fr;gap:10px;margin-top:8px}@media(min-width:860px){.zonegrid{grid-template-columns:1fr 1fr}}");
  html += F(".zrow{border:1px solid var(--line);background:rgba(255,255,255,.66);border-radius:14px;padding:12px;animation:rise .5s ease-out}html[data-theme='dark'] .zrow{background:rgba(15,31,44,.7)}");
  html += F(".zrow:nth-child(2n){animation-delay:.04s}.actions{display:flex;gap:8px;margin-top:12px;flex-wrap:wrap}");
  html += F(".btn{padding:10px 14px;border:0;border-radius:12px;background:linear-gradient(180deg,var(--pri),var(--pri2));color:#fff;cursor:pointer;font-weight:800;transition:.18s transform,.18s box-shadow,.18s opacity}");
  html += F(".link{display:inline-flex;align-items:center;justify-content:center;padding:10px 14px;border-radius:12px;border:1px solid var(--line);background:rgba(255,255,255,.72);color:var(--text);text-decoration:none;font-weight:800;backdrop-filter:blur(8px)}");
  html += F("html[data-theme='dark'] .link{background:rgba(15,31,44,.72)}@media(max-width:640px){body{padding:14px}.title{font-size:1.3rem}.head-actions{width:100%}.head-actions .btn-link,.head-actions .btn-ghost{flex:1}}");
  html += F("</style></head><body>");

  html += F("<div class='wrap'>");
  html += F("<div class='head'><div><div class='title'>Setup</div><div class='sub'>Controller and pin configuration</div></div><div class='head-actions'><button class='btn-ghost' type='button' id='themeToggle'>Theme</button><a class='btn-link' href='/'>Dashboard</a></div></div>");
  html += F("<div class='card'>");
  html += F("<div class='pills'><div class='pill'><b>Host</b> 8266Irrigation.local</div><div class='pill'><b>IP</b> ");
  html += WiFi.localIP().toString();
  html += F("</div><div class='pill'><b>WiFi</b> ");
  html += (WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  html += F("</div></div>");
  html += F("<h2>General</h2><form action='/configure' method='POST'>");
  html += F("<div class='row'><label>Zone Count (1..6)</label><input type='number' name='zonesCount' min='1' max='6' value='");
  html += String(zonesCount);
  html += F("'></div><div class='row'><span class='help'>Set zone pin to -1 for unused zones.</span></div>");

  html += F("<div class='row'><label>UTC Offset (hours)</label><input type='number' name='tz' min='-12' max='14' step='0.5' value='");
  html += String(tzOffsetHours, 2);
  html += F("'></div><div class='row'><span class='help'>Used for schedule time calculations.</span></div>");

  html += F("<h2>Weather Rules</h2>");
  html += F("<div class='row'><label><input type='checkbox' name='wx_en' ");
  html += (weatherEnabled ? "checked" : "");
  html += F("> Enable Weather</label></div>");
  html += F("<div class='row'><label>Latitude</label><input type='number' name='wx_lat' min='-90' max='90' step='0.000001' value='");
  html += String(weatherLat, 6);
  html += F("'><label>Longitude</label><input type='number' name='wx_lon' min='-180' max='180' step='0.000001' value='");
  html += String(weatherLon, 6);
  html += F("'></div>");
  html += F("<div class='row'><label><input type='checkbox' name='wind_en' ");
  html += (windDelayEnabled ? "checked" : "");
  html += F("> Wind threshold delay</label><label>Threshold km/h</label><input type='number' name='wind_thr' min='0' max='200' step='0.5' value='");
  html += String(windSpeedThreshold, 1);
  html += F("'></div>");
  html += F("<div class='row'><label><input type='checkbox' name='rain_cancel' ");
  html += (rainCancelEnabled ? "checked" : "");
  html += F("> Cancel queued/running if raining</label></div>");
  html += F("<div class='row'><span class='help'>Source: Open-Meteo current weather (no API key).</span></div>");

  html += F("<h2>Zone Pins</h2>");
  html += F("<div class='zonegrid'>");
  for (int z = 0; z < (int)MAX_ZONES; z++) {
    html += F("<div class='zrow'><div class='row'><label>Zone ");
    html += String(z + 1);
    html += F(" Pin</label><input type='number' name='pin");
    html += String(z);
    html += F("' min='-1' max='16' value='");
    html += String(zonePins[z]);
    html += F("'><label>Name</label><input type='text' name='zname");
    html += String(z);
    html += F("' value='");
    html += zoneNames[z];
    html += F("'></div></div>");
  }
  html += F("</div>");

  html += F("<div class='actions'><button class='btn' type='submit'>Save Setup</button><a class='link' href='/'>Back</a></div>");
  html += F("</form></div></div><script>(function(){const k='ui_theme';let r=localStorage.getItem(k);if(!r){r=(window.matchMedia&&window.matchMedia('(prefers-color-scheme: dark)').matches)?'dark':'light';}document.documentElement.setAttribute('data-theme',r);");
  html += F("const b=document.getElementById('themeToggle');if(b){b.addEventListener('click',()=>{const c=document.documentElement.getAttribute('data-theme')==='dark'?'dark':'light';");
  html += F("const n=(c==='dark')?'light':'dark';document.documentElement.setAttribute('data-theme',n);localStorage.setItem(k,n);});}})();</script></body></html>");

  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcdShow("Smart Irrigation", "Booting...");

  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }

  loadConfig();
  loadSchedule();
  initZonePins();
  allZonesOff();

  WiFi.hostname("8266Irrigation");

  wifiManager.setTimeout(180);
  if (!wifiManager.autoConnect("ESPIrrigationAP")) {
    ESP.restart();
  }
  MDNS.begin("8266Irrigation");
  MDNS.addService("http", "tcp", 80);

  applyTimeConfig();

  ArduinoOTA.setHostname("8266Irrigation");
  ArduinoOTA.begin();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/submit", HTTP_POST, handleSubmit);
  server.on("/setup", HTTP_GET, handleSetupPage);
  server.on("/configure", HTTP_POST, handleConfigure);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/valve/on", HTTP_POST, handleValveOn);
  server.on("/valve/off", HTTP_POST, handleValveOff);

  server.begin();

  lcdShow("WiFi Connected", "8266Irrigation");
  delay(1500);
}

void loop() {
  ArduinoOTA.handle();
  MDNS.update();
  server.handleClient();

  const uint32_t nowMs = millis();

  if (nowMs - lastTickMs >= 1000UL) {
    lastTickMs = nowMs;

    updateWeatherCache(nowMs);
    evaluateWeatherBlocks();

    time_t now = time(nullptr);
    if (now > 0) {
      time_t currentMinute = minuteEpoch(now);
      if (lastSchedulerMinuteEpoch == 0 || currentMinute < lastSchedulerMinuteEpoch) {
        lastSchedulerMinuteEpoch = currentMinute - 60;
      }

      time_t minuteToProcess = lastSchedulerMinuteEpoch + 60;
      uint8_t processed = 0;
      while (minuteToProcess <= currentMinute && processed < 10) {
        processDueStartsAtMinute(minuteToProcess);
        lastSchedulerMinuteEpoch = minuteToProcess;
        minuteToProcess += 60;
        processed++;
      }
    }

    for (int z = 0; z < (int)zonesCount; z++) {
      if (zoneActive[z] && hasDurationCompleted(z)) {
        turnOffZone(z);
      }
    }

    drainQueue();
  }

  if (nowMs - lastLcdMs >= 1000UL) {
    lastLcdMs = nowMs;
    lcdHome();
  }
}
