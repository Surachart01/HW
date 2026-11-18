/*
  ระบบฟาร์มอัจฉริยะ IoT สำหรับพืชอวบน้ำ — เฟิร์มแวร์ D1 Mini ESP8266
  ขาต่อตามตารางที่ 3-1 / ภาพที่ 3-2
    BME280, BH1750 : SDA = D2, SCL = D1 (I2C ร่วมกัน)
    Soil Moisture  : A0
    รีเลย์ปั๊ม (IN1) : D5     รีเลย์ไฟ LED (IN2) : D6

  ไลบรารีที่ต้องติดตั้ง (Library Manager):
    Adafruit BME280 Library, BH1750 (by Christopher Laws), ArduinoJson (v7)
  บอร์ด: LOLIN(WEMOS) D1 R2 & mini  (แพ็กเกจ esp8266 by ESP8266 Community)

  การทำงานทุก 60 วินาที (ภาพที่ 3-3):
    ตรวจ Wi-Fi -> อ่านค่าเซ็นเซอร์ -> ส่งข้อมูลและรับค่าตั้ง -> ควบคุมปั๊มและไฟ
  ถ้าเน็ตหลุด ระบบยังควบคุมตามเกณฑ์ล่าสุดต่อไปได้ และจะต่อ Wi-Fi ใหม่เอง
*/
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <ArduinoJson.h>
#include <time.h>
#include "config.h"

// ถ้ามีไฟล์ model.h (ได้จาก ai/train_model.py) อยู่ในโฟลเดอร์เดียวกัน จะเปิดโหมด AI ให้อัตโนมัติ
#if __has_include("model.h")
  #include "model.h"
  #define HAS_MODEL 1
#else
  #define HAS_MODEL 0
#endif

const uint8_t PIN_PUMP = D5;
const uint8_t PIN_LED  = D6;
const uint8_t PIN_SOIL = A0;

// ---------------- ค่าตั้ง (ค่าเริ่มต้นตามตารางที่ 3-3 และ 3-6; เซิร์ฟเวอร์ส่งค่าใหม่มาแทนได้) ----------------
struct Settings {
  char  mode[8]      = "auto";  // auto = ตามเกณฑ์, ai = โหมด AI, manual = สั่งเองจากเว็บ
  float soilOn       = 30;      // ความชื้นต่ำกว่านี้ -> เริ่มให้น้ำ
  float soilOff      = 45;      // ให้น้ำจนถึงค่านี้
  float luxOn        = 2000;    // กลางวันและแสงต่ำกว่านี้ -> เปิดไฟ
  float luxOff       = 3000;    // แสงเกินนี้ -> ปิดไฟ
  int   dayStart     = 6;       // กลางวัน 06.00–18.00 น.
  int   dayEnd       = 18;
  int   pumpPulseSec = 5;       // เปิดปั๊ม 5 วินาทีต่อรอบ
  int   maxPulsesDay = 40;      // ป้องกันเพิ่มเติม: เปิดปั๊มได้ไม่เกินกี่ครั้งต่อวัน
  float aiPredMin    = 30;      // ค่าพยากรณ์ 3 ชม. ต่ำกว่านี้ -> ให้น้ำล่วงหน้าช่วงเช้า
  float aiEmergency  = 20;      // ความชื้นปัจจุบันต่ำกว่านี้ -> ให้น้ำทันที
  int   aiStart      = 6;       // ช่วงให้น้ำล่วงหน้า 06.00–10.00 น.
  int   aiEnd        = 10;
} cfg;

struct Reading {
  float t = NAN, rh = NAN, p = NAN, lux = NAN, soil = NAN;
  int   soilRaw = -1;
  bool  okBme = false, okLux = false, okSoil = false;
  bool ok() const { return okBme && okLux && okSoil; }
};

Adafruit_BME280 bme;
BH1750 lightMeter(0x23);
bool bmeReady = false, luxReady = false;

// ---------------- สถานะ ----------------
bool pumpOn = false, ledOn = false, ledManual = false, watering = false;
unsigned long pumpOffAt = 0, lastCycle = 0, lastWifiTry = 0;
unsigned long lastWaterMs = 0;      // millis() ตอนให้น้ำครั้งล่าสุด (0 = ยังไม่รู้)
long  serverWaterAgoS = -1;         // เวลาตั้งแต่ให้น้ำล่าสุดที่เซิร์ฟเวอร์บอก (ใช้หลังรีบูต)
int   pulsesToday = 0, pulsesToReport = 0, today = -1;
const char* pumpSource = "auto";
const char* decision = "start";
bool  firstCycle = true, serverOk = false;

// ---------------- ค่าเฉลี่ย 10 นาที สำหรับโมเดล AI (หัวข้อ 3.4.6 ข้อ 2) ----------------
struct Avg { float t, rh, p, lux, soil; };
Avg   ring[7];                       // ย้อนหลัง 1 ชั่วโมง = 7 ช่อง (ปัจจุบัน + 6 ช่องก่อนหน้า)
int   ringCount = 0;
Avg   sum = {0, 0, 0, 0, 0};
int   sumCount = 0, cyclesInWindow = 0;
float prediction = NAN;              // ความชื้นที่พยากรณ์ในอีก 3 ชม. (NAN = ยังไม่มี)
unsigned long predictionAt = 0;

// ======================================================================
void relayWrite(uint8_t pin, bool on) {
  digitalWrite(pin, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setPump(bool on) {
  if (on && !pumpOn) {
    unsigned long sec = constrain(cfg.pumpPulseSec, 1, 10);   // ป้องกัน: เปิดได้สูงสุด 10 วินาทีต่อครั้ง
    pumpOffAt = millis() + sec * 1000UL;
    lastWaterMs = millis(); if (lastWaterMs == 0) lastWaterMs = 1;
    serverWaterAgoS = -1;
    pulsesToday++; pulsesToReport++;
  }
  pumpOn = on;
  relayWrite(PIN_PUMP, on);
}

void setLed(bool on) {
  ledOn = on;
  relayWrite(PIN_LED, on);
}

// ---------------- เวลา ----------------
bool timeValid() { return time(nullptr) > 1700000000; }

float hourNow() {                     // ชั่วโมงแบบทศนิยม เช่น 13.5 = 13.30 น. (-1 = ยังไม่ได้เวลาจาก NTP)
  if (!timeValid()) return -1;
  time_t now = time(nullptr);
  struct tm* lt = localtime(&now);
  return lt->tm_hour + lt->tm_min / 60.0f;
}

int dayOfYear() {
  if (!timeValid()) return (int)(millis() / 86400000UL);
  time_t now = time(nullptr);
  return localtime(&now)->tm_yday;
}

// ---------------- Wi-Fi ----------------
void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  if (millis() - lastWifiTry < 30000UL && lastWifiTry != 0) return;   // ลองใหม่ทุก 30 วินาที ไม่บล็อกการควบคุม
  lastWifiTry = millis();
  Serial.println(F("[WiFi] กำลังเชื่อมต่อใหม่..."));
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

// ---------------- เซ็นเซอร์ ----------------
void initSensors() {
  if (!bmeReady) bmeReady = bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire);
  if (!luxReady) luxReady = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
}

Reading readSensors() {
  Reading r;
  initSensors();                               // ถ้าเซ็นเซอร์หลุดแล้วต่อกลับ จะเริ่มใหม่ได้เอง
  if (bmeReady) {
    r.t  = bme.readTemperature();
    r.rh = bme.readHumidity();
    r.p  = bme.readPressure() / 100.0f;
    r.okBme = !isnan(r.t) && !isnan(r.rh) && !isnan(r.p) &&
              r.t > -10 && r.t < 60 && r.rh >= 0 && r.rh <= 100 && r.p > 800 && r.p < 1100;
    if (!r.okBme) bmeReady = false;
  }
  if (luxReady) {
    r.lux = lightMeter.readLightLevel();
    r.okLux = r.lux >= 0 && r.lux < 65536;
    if (!r.okLux) luxReady = false;
  }
  long acc = 0;
  for (int i = 0; i < 10; i++) { acc += analogRead(PIN_SOIL); delay(5); }
  r.soilRaw = acc / 10;
  // สายหลุดหรือเซ็นเซอร์เสีย ค่ามักจะชิด 0 หรือ 1023
  r.okSoil = r.soilRaw > 30 && r.soilRaw < 1010;
  if (r.okSoil) {
    float pct = (float)(SOIL_RAW_DRY - r.soilRaw) * 100.0f / (SOIL_RAW_DRY - SOIL_RAW_WET);
    r.soil = constrain(pct, 0.0f, 100.0f);
  }
  return r;
}

// ---------------- AI: ค่าเฉลี่ย 10 นาทีและการพยากรณ์ ----------------
float hoursSinceWater() {
  if (lastWaterMs != 0) return (millis() - lastWaterMs) / 3600000.0f;
  if (serverWaterAgoS >= 0) return serverWaterAgoS / 3600.0f;
  return 72;                                  // ไม่ทราบ -> ใช้ค่าสูงสุดตามที่ฝึกโมเดล
}

void updateAverages(const Reading& r) {
  if (r.ok()) {
    sum.t += r.t; sum.rh += r.rh; sum.p += r.p; sum.lux += r.lux; sum.soil += r.soil; sumCount++;
  }
  cyclesInWindow++;
  if ((unsigned long)cyclesInWindow * CYCLE_SECONDS < 600) return;   // ยังไม่ครบ 10 นาที

  if (sumCount >= cyclesInWindow / 2) {
    Avg a = { sum.t / sumCount, sum.rh / sumCount, sum.p / sumCount, sum.lux / sumCount, sum.soil / sumCount };
    if (ringCount == 7) { for (int i = 0; i < 6; i++) ring[i] = ring[i + 1]; ringCount = 6; }
    ring[ringCount++] = a;
  } else {
    ringCount = 0;                            // ข้อมูลขาดช่วง -> เริ่มนับ 1 ชั่วโมงใหม่ (กลไกสำรอง)
  }
  sum = {0, 0, 0, 0, 0}; sumCount = 0; cyclesInWindow = 0;

#if HAS_MODEL
  float h = hourNow();
  if (ringCount == 7 && h >= 0) {
    const Avg& now = ring[6];
    double in[9] = { now.t, now.rh, now.p, now.lux, now.soil,
                     now.soil - ring[0].soil,
                     sin(2 * PI * h / 24.0), cos(2 * PI * h / 24.0),
                     min(hoursSinceWater(), 72.0f) };
    double y = score(in);                     // ลำดับตรงกับ FEATURES ใน train_model.py
    prediction = (y >= 0 && y <= 100) ? (float)y : NAN;
    predictionAt = millis();
  } else {
    prediction = NAN;
  }
#endif
}

bool aiReady() {
  return HAS_MODEL && !isnan(prediction) && millis() - predictionAt < 15UL * 60000UL;
}

// ---------------- ส่งข้อมูลและรับค่าตั้ง ----------------
void applySettings(JsonObjectConst s) {
  if (s.isNull()) return;
  const char* m = s["mode"] | cfg.mode;
  if (!strcmp(m, "auto") || !strcmp(m, "ai") || !strcmp(m, "manual")) strlcpy(cfg.mode, m, sizeof(cfg.mode));
  cfg.soilOn       = s["soil_on"]        | cfg.soilOn;
  cfg.soilOff      = s["soil_off"]       | cfg.soilOff;
  cfg.luxOn        = s["lux_on"]         | cfg.luxOn;
  cfg.luxOff       = s["lux_off"]        | cfg.luxOff;
  cfg.dayStart     = s["day_start"]      | cfg.dayStart;
  cfg.dayEnd       = s["day_end"]        | cfg.dayEnd;
  cfg.pumpPulseSec = s["pump_pulse_s"]   | cfg.pumpPulseSec;
  cfg.maxPulsesDay = s["max_pulses_day"] | cfg.maxPulsesDay;
  cfg.aiPredMin    = s["ai_pred_min"]    | cfg.aiPredMin;
  cfg.aiEmergency  = s["ai_emergency"]   | cfg.aiEmergency;
  cfg.aiStart      = s["ai_start"]       | cfg.aiStart;
  cfg.aiEnd        = s["ai_end"]         | cfg.aiEnd;
}

void runCommand(JsonObjectConst c, const Reading& r) {
  if (c.isNull()) return;
  const char* dev = c["device"] | "";
  const char* act = c["action"] | "";
  Serial.printf("[คำสั่ง] %s %s\n", dev, act);
  if (!strcmp(dev, "pump") && !strcmp(act, "pulse")) {
    if (r.okSoil) { pumpSource = "manual"; setPump(true); decision = "manual_pulse"; }
    else decision = "manual_blocked_sensor_error";
  } else if (!strcmp(dev, "led")) {
    ledManual = !strcmp(act, "on");
  }
}

void putNum(JsonDocument& d, const char* k, float v, bool ok, int digits) {
  if (ok && !isnan(v)) d[k] = serialized(String(v, digits)); else d[k] = nullptr;
}

bool postReading(const Reading& r, JsonDocument& resp) {
  if (WiFi.status() != WL_CONNECTED) return false;
  JsonDocument d;
  d["device"] = DEVICE_ID;
  putNum(d, "temperature", r.t, r.okBme, 2);
  putNum(d, "humidity", r.rh, r.okBme, 2);
  putNum(d, "pressure", r.p, r.okBme, 2);
  putNum(d, "lux", r.lux, r.okLux, 1);
  putNum(d, "soil", r.soil, r.okSoil, 2);
  d["soil_raw"] = r.soilRaw;
  d["pump"] = pulsesToReport > 0 ? 1 : 0;      // มีการให้น้ำนับจากรายงานครั้งก่อน
  d["pump_pulses"] = pulsesToReport;
  d["pump_source"] = pumpSource;
  d["led"] = ledOn ? 1 : 0;
  d["mode"] = cfg.mode;
  putNum(d, "soil_pred_3h", prediction, aiReady(), 2);
  d["decision"] = decision;
  JsonArray err = d["errors"].to<JsonArray>();
  if (!r.okBme) err.add("bme280");
  if (!r.okLux) err.add("bh1750");
  if (!r.okSoil) err.add("soil");
  d["rssi"] = WiFi.RSSI();
  d["uptime_s"] = millis() / 1000;
  d["has_model"] = HAS_MODEL;
  String body; serializeJson(d, body);

  HTTPClient http;
  WiFiClient plain;
  std::unique_ptr<BearSSL::WiFiClientSecure> secure;
  String url = String(SERVER_URL) + "/api/readings";
  bool ok;
  if (url.startsWith("https")) {
    secure.reset(new BearSSL::WiFiClientSecure);
    secure->setInsecure();                     // ไม่ตรวจใบรับรอง (พอสำหรับต้นแบบ)
    secure->setBufferSizes(4096, 1024);
    ok = http.begin(*secure, url);
  } else {
    ok = http.begin(plain, url);
  }
  if (!ok) return false;
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("x-api-key", API_KEY);
  int code = http.POST(body);
  bool success = false;
  if (code == 200 || code == 201) {
    success = !deserializeJson(resp, http.getString());
    if (success) pulsesToReport = 0;          // ส่งไม่สำเร็จ -> เก็บไว้รายงานรอบหน้า
  } else {
    Serial.printf("[HTTP] ส่งไม่สำเร็จ code=%d %s\n", code, http.errorToString(code).c_str());
  }
  http.end();
  return success;
}

// ---------------- ควบคุม (ตารางที่ 3-3 และ 3-6) ----------------
void control(const Reading& r) {
  float h = hourNow();
  bool day = h >= 0 && h >= cfg.dayStart && h < cfg.dayEnd;

  if (!strcmp(cfg.mode, "manual")) {          // โหมดสั่งเอง: ไฟตามปุ่มบนเว็บ, ปั๊มเฉพาะเมื่อกดสั่ง
    setLed(ledManual);
    watering = false;
    decision = "manual_mode";
    return;
  }

  if (!r.ok()) {                              // อ่านค่าไม่ได้ -> ตัดรีเลย์ทั้งหมด
    setPump(false); setLed(false); watering = false;
    decision = "sensor_error";
    return;
  }

  // ไฟ LED: กลางวัน + hysteresis 2,000 / 3,000 lux
  if (!day) setLed(false);
  else if (r.lux < cfg.luxOn) setLed(true);
  else if (r.lux > cfg.luxOff) setLed(false);

  // ปั๊ม
  bool useAi = !strcmp(cfg.mode, "ai") && aiReady();
  if (useAi) {
    if (r.soil < cfg.aiEmergency) { watering = true; decision = "ai_emergency"; }
    else if (!watering && prediction < cfg.aiPredMin) {
      if (h >= cfg.aiStart && h < cfg.aiEnd) { watering = true; decision = "ai_prewater"; }
      else decision = "ai_wait_morning";
    } else if (!watering) decision = "ai_no_water";
  } else {
    if (!strcmp(cfg.mode, "ai")) decision = "ai_fallback_threshold";
    else decision = "threshold";
    if (r.soil < cfg.soilOn) watering = true;
  }
  if (watering && r.soil >= cfg.soilOff) watering = false;

  if (watering) {
    if (pulsesToday < cfg.maxPulsesDay) { pumpSource = useAi ? "ai" : "auto"; setPump(true); }
    else { watering = false; decision = "daily_limit"; }
  }
}

// ======================================================================
void cycle() {
  int d = dayOfYear();
  if (d != today) { today = d; pulsesToday = 0; }

  Reading r = readSensors();
  updateAverages(r);

  JsonDocument resp;
  serverOk = postReading(r, resp);
  if (serverOk) {
    applySettings(resp["settings"]);
    if (!resp["last_water_ago_s"].isNull() && lastWaterMs == 0) serverWaterAgoS = resp["last_water_ago_s"];
  }
  decision = "-";
  control(r);
  if (serverOk) runCommand(resp["command"], r);

  Serial.printf("T=%.1fC RH=%.0f%% P=%.0fhPa Lux=%.0f Soil=%.0f%% (raw %d) | ปั๊ม=%s ไฟ=%s | โหมด=%s %s | pred3h=%s | WiFi=%s server=%s\n",
                r.t, r.rh, r.p, r.lux, r.soil, r.soilRaw,
                pumpOn ? "ON" : "off", ledOn ? "ON" : "off", cfg.mode, decision,
                isnan(prediction) ? "-" : String(prediction, 1).c_str(),
                WiFi.status() == WL_CONNECTED ? "ok" : "หลุด", serverOk ? "ok" : "ไม่ตอบ");
}

void setup() {
  // ตั้งขารีเลย์ให้ "ปิด" ก่อนเป็น OUTPUT ป้องกันปั๊มกระตุกตอนเปิดเครื่อง
  digitalWrite(PIN_PUMP, RELAY_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(PIN_LED,  RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  setPump(false); setLed(false);
  pulsesToday = 0; pulsesToReport = 0; lastWaterMs = 0;

  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== Smart Farm (D1 Mini) ==="));
  Serial.printf("โมเดล AI: %s\n", HAS_MODEL ? "มี (model.h)" : "ไม่มี — ใช้โหมดเกณฑ์");

  Wire.begin(D2, D1);                         // SDA = D2, SCL = D1
  initSensors();
  Serial.printf("BME280: %s  BH1750: %s\n", bmeReady ? "พบ" : "ไม่พบ!", luxReady ? "พบ" : "ไม่พบ!");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  lastWifiTry = millis();
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) Serial.printf("[WiFi] เชื่อมต่อแล้ว IP=%s\n", WiFi.localIP().toString().c_str());
  else Serial.println(F("[WiFi] ยังเชื่อมต่อไม่ได้ จะลองใหม่ระหว่างทำงาน"));

  configTime(TZ_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");
}

void loop() {
  if (pumpOn && (long)(millis() - pumpOffAt) >= 0) setPump(false);   // ตัดปั๊มเมื่อครบเวลา
  ensureWifi();
  if (firstCycle || millis() - lastCycle >= CYCLE_SECONDS * 1000UL) {
    firstCycle = false;
    lastCycle = millis();
    cycle();
  }
  delay(20);
}
