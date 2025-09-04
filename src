/*
  ESP32 OLED + MQTT + DS18B20 + Simple FAN ON/OFF (with hysteresis)
  -----------------------------------------------------------------
  Display (SSD1306 128x64), 3 rows, two left-aligned columns:

    Row 1:  <MQTT Temp °C>               <MQTT Humidity %>
    Row 2:  <MQTT Pressure hPa>          FAN <ON|OFF>
    Row 3:  <DS18B20 Temp (integer) °C>  <Last MQTT update HH:MM>

  Notes
  - The time shown is the time of the last *incoming MQTT payload*, not “current time”.
  - DS18B20 is sampled every DS18B20_SAMPLE_MS (blocking requestTemperatures()).
  - FAN is low-side driven by an N-MOSFET from a GPIO (HIGH = ON).
  - Hysteresis: FAN_T_ON_C > FAN_T_OFF_C (compile-time enforced).

  MQTT Topics
  - Inbound (from Home Assistant / Mosquitto) – values for the OLED:
      esp32/weather/temperature  -> e.g. "23.4"
      esp32/weather/humidity     -> e.g. "55"
      esp32/weather/pressure     -> e.g. "1015"
  - Outbound (from ESP32, retained):
      esp32/fan/temperature      -> DS18B20 temperature (e.g. "23.75")
      esp32/fan/state            -> "ON" | "OFF"
  - Control (from HA to ESP32) + retained config state:
      esp32/fan/cmd              -> "AUTO" | "ON" | "OFF"
      esp32/fan/mode             -> retained mode ("AUTO"/"ON"/"OFF")
      esp32/fan/config/on_c/set  -> float °C (command)
      esp32/fan/config/off_c/set -> float °C (command)
      esp32/fan/config/on_c      -> retained float °C (state)
      esp32/fan/config/off_c     -> retained float °C (state)
  - Availability (LWT):
      esp32/availability         -> "online" | "offline"
*/

#if !defined(ARDUINO_ARCH_ESP32)
#error "This sketch targets ESP32 (Arduino core). Please select an ESP32 board."
#endif

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ctype.h>   // toupper()
#include <math.h>    // isfinite(), fabsf()

// ============================================================================
// SECTION: USER CONFIGURATION
// ============================================================================

// --- Wi-Fi ---
constexpr const char* WIFI_SSID     = "<your-ssid>";
constexpr const char* WIFI_PASSWORD = "<your-pass>";

// --- MQTT ---
constexpr const char* MQTT_SERVER   = "<broker-ip>";
constexpr uint16_t    MQTT_PORT     = <port>;
constexpr const char* MQTT_USER     = "<user>";
constexpr const char* MQTT_PASSWORD = "<pass>";

// --- MQTT topics (inbound from HA -> values shown on OLED) ---
constexpr const char* TOPIC_TEMP_IN  = "esp32/weather/temperature";
constexpr const char* TOPIC_HUMI_IN  = "esp32/weather/humidity";
constexpr const char* TOPIC_PRESS_IN = "esp32/weather/pressure";

// --- MQTT topics (outbound from ESP32 -> HA, retained) ---
constexpr const char* TOPIC_DS18_OUT = "esp32/fan/temperature";
constexpr const char* TOPIC_FAN_OUT  = "esp32/fan/state";

// --- MQTT control + retained config state ---
constexpr const char* TOPIC_FAN_CMD        = "esp32/fan/cmd";              // "AUTO"|"ON"|"OFF"
constexpr const char* TOPIC_FAN_MODE_STATE = "esp32/fan/mode";             // retained: "AUTO"/"ON"/"OFF"
constexpr const char* TOPIC_ON_C_STATE     = "esp32/fan/config/on_c";      // retained float
constexpr const char* TOPIC_ON_C_SET       = "esp32/fan/config/on_c/set";  // command float
constexpr const char* TOPIC_OFF_C_STATE    = "esp32/fan/config/off_c";     // retained float
constexpr const char* TOPIC_OFF_C_SET      = "esp32/fan/config/off_c/set"; // command float

// --- MQTT availability (LWT) ---
constexpr const char* TOPIC_AVAIL   = "esp32/availability";
constexpr const char* AVAIL_ONLINE  = "online";
constexpr const char* AVAIL_OFFLINE = "offline";

// --- OLED display & layout ---
constexpr uint8_t  SCREEN_WIDTH  = 128;
constexpr uint8_t  SCREEN_HEIGHT = 64;
constexpr uint8_t  OLED_ADDR     = 0x3C;

// --- Column X positions (both columns are left-aligned) ---
constexpr int16_t COL1_X = 0;
constexpr int16_t COL2_X = 80;   // tweak if you need more/less separation

// --- Row Y positions ---
constexpr int16_t ROW_Y0 = 2;
constexpr int16_t ROW_Y1 = 22;
constexpr int16_t ROW_Y2 = 42;

// --- Time & redraw ---
constexpr long     TIMEZONE_OFFSET    = 7200;      // GMT+2
constexpr uint32_t NTP_POLL_INTERVAL  = 60000UL;   // 60 s
constexpr uint32_t REDRAW_INTERVAL_MS = 300000UL;  // safety redraw each 5 min


// --- Pins (ESP32 Dev Module) ---
constexpr uint8_t PIN_I2C_SDA = 21;   // OLED SDA
constexpr uint8_t PIN_I2C_SCL = 22;   // OLED SCL
constexpr uint8_t PIN_ONEWIRE = 4;    // DS18B20 DATA (with 4.7k pull-up to 3V3)
constexpr uint8_t PIN_FAN     = 23;   // MOSFET gate (LOW=OFF, HIGH=ON)

// --- Hysteresis defaults (runtime-changeable via MQTT) ---
constexpr float FAN_T_ON_C_DEFAULT  = 30.0f;      // turn ON at/above
constexpr float FAN_T_OFF_C_DEFAULT = 25.0f;      // turn OFF at/below
static_assert(FAN_T_ON_C_DEFAULT > FAN_T_OFF_C_DEFAULT, "ON must be > OFF");

// --- Sampling ---
constexpr uint32_t DS18B20_SAMPLE_MS = 2000UL;    // read every 2 seconds

// ============================================================================
// SECTION: GLOBAL OBJECTS
// ============================================================================

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
WiFiClient      netClient;
PubSubClient    mqtt(netClient);

// --- NTP ---
WiFiUDP  ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", TIMEZONE_OFFSET, NTP_POLL_INTERVAL);

// --- DS18B20 ---
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);

// ============================================================================
// SECTION: RUNTIME STATE
// ============================================================================
// --- Inbound HA values (as formatted strings for OLED) ---
String haTemp  = "--";
String haHumi  = "--";
String haPress = "--";

// --- Local sensor & control ---
float dsTempC = NAN;
bool  fanOn   = false;
float fanOnC        = FAN_T_ON_C_DEFAULT;   // mutable via MQTT
float fanOffC       = FAN_T_OFF_C_DEFAULT;  // mutable via MQTT
enum class FanMode { AUTO, FORCED_ON, FORCED_OFF };
FanMode fanMode     = FanMode::AUTO;

// --- Last time (HH:MM) when any HA payload arrived (displayed on row 3, col 2) ---
String lastUpdateHHMM = "--:--";

// --- Redraw flag ---
volatile bool hasFreshData = false;

// ============================================================================
/* SECTION: HELPERS (formatting, time, publishing, control) */
// ============================================================================

/**
 * @brief Parse the first float from a payload and format with given precision.
 *        No left-padding is applied (OLED uses left-aligned columns).
 * @param payload  Input string (MQTT payload).
 * @param digits   Number of decimal places.
 * @return Formatted string or "--" if parsing failed.
 */
String fmtNumber(const String& payload, int digits) {
  float v = NAN;
  if (sscanf(payload.c_str(), "%f", &v) == 1 && isfinite(v)) {
    return String(v, digits);
  }
  return "--";
}

/**
 * @brief Get local HH:MM from NTP (or "--:--" until time is set).
 */
String nowHHMM() {
  if (!timeClient.isTimeSet()) return "--:--";
  const String hhmmss = timeClient.getFormattedTime(); // "HH:MM:SS"
  return hhmmss.substring(0, 5);
}

/**
 * @brief Remember "last updated" time for inbound MQTT payloads.
 */
inline void stampLastUpdate() { lastUpdateHHMM = nowHHMM(); }

/**
 * @brief Publish fan ON/OFF state (retained).
 */
void publishFanState() {
  mqtt.publish(TOPIC_FAN_OUT, fanOn ? "ON" : "OFF", true);
}

/**
 * @brief Publish current fan mode (retained).
 */
void publishFanMode() {
  const char* m = (fanMode == FanMode::AUTO) ? "AUTO" : (fanMode == FanMode::FORCED_ON ? "ON" : "OFF");
  mqtt.publish(TOPIC_FAN_MODE_STATE, m, true);
}

/**
 * @brief Publish current hysteresis thresholds (retained).
 */
void publishFanConfig() {
  char b1[16], b2[16];
  dtostrf(fanOnC,  0, 2, b1);
  dtostrf(fanOffC, 0, 2, b2);
  mqtt.publish(TOPIC_ON_C_STATE,  b1, true);
  mqtt.publish(TOPIC_OFF_C_STATE, b2, true);
}

/**
 * @brief Validate DS18B20 reading: rejects disconnected (-127 °C) and power-on default (85 °C).
 */
inline bool isValidDs18(float t) {
  return isfinite(t) && t > -55.0f && t < 125.0f && fabsf(t - 85.0f) > 0.001f;
}

/**
 * @brief Publish DS18B20 temperature if last cached value is valid (retained).
 */
void publishDs18IfValid() {
  if (isValidDs18(dsTempC)) {
    char tb[16]; 
    dtostrf(dsTempC, 0, 2, tb);
    mqtt.publish(TOPIC_DS18_OUT, tb, true);
  }
}

/**
 * @brief Decide target fan state (mode + hysteresis) and drive GPIO if it changed.
 */
void applyFanControl() {
  bool targetOn = fanOn;

  if (fanMode == FanMode::FORCED_ON)       targetOn = true;
  else if (fanMode == FanMode::FORCED_OFF) targetOn = false;
  else {
    // AUTO with hysteresis (only when sensor value is valid)
    if (isValidDs18(dsTempC)) {
      if (!fanOn && dsTempC >= fanOnC)      targetOn = true;
      else if (fanOn && dsTempC <= fanOffC) targetOn = false;
    }
  }

  if (targetOn != fanOn) {
    fanOn = targetOn;
    digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
    publishFanState();
  }
}

/**
 * @brief Read DS18B20 (blocking), update cache on valid value, publish, and apply fan control.
 */
void readDs18b20() {
  sensors.requestTemperatures();                // blocking; OK at 2 s cadence
  float t = sensors.getTempCByIndex(0);
  if (isValidDs18(t)) dsTempC = t;              // cache only valid reading
  publishDs18IfValid();                         // publishes last known good
  applyFanControl();
}

// ============================================================================
// SECTION: MQTT (subscribe, callback, connection with LWT)
// ============================================================================

/**
 * @brief Subscribe to all inbound topics (HA values + control).
 */
void subscribeTopics() {
  mqtt.subscribe(TOPIC_TEMP_IN);
  mqtt.subscribe(TOPIC_HUMI_IN);
  mqtt.subscribe(TOPIC_PRESS_IN);
  mqtt.subscribe(TOPIC_FAN_CMD);
  mqtt.subscribe(TOPIC_ON_C_SET);
  mqtt.subscribe(TOPIC_OFF_C_SET);
}

/**
 * @brief Uppercase in-place (ASCII).
 */
static void to_upper_inplace(String& s) {
  for (size_t i = 0; i < s.length(); ++i)
    s.setCharAt(i, toupper((unsigned char)s[i]));
}

/**
 * @brief MQTT callback: HA → values, mode changes, threshold updates.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg; msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  if      (strcmp(topic, TOPIC_TEMP_IN)  == 0) { haTemp  = fmtNumber(msg, 1); stampLastUpdate(); }
  else if (strcmp(topic, TOPIC_HUMI_IN)  == 0) { haHumi  = fmtNumber(msg, 0); stampLastUpdate(); }
  else if (strcmp(topic, TOPIC_PRESS_IN) == 0) { haPress = fmtNumber(msg, 0); stampLastUpdate(); }

  else if (strcmp(topic, TOPIC_FAN_CMD) == 0) {
    String m = msg; to_upper_inplace(m);
    if      (m == "AUTO") { fanMode = FanMode::AUTO;       publishFanMode(); applyFanControl(); }
    else if (m == "ON")   { fanMode = FanMode::FORCED_ON;  publishFanMode(); applyFanControl(); }
    else if (m == "OFF")  { fanMode = FanMode::FORCED_OFF; publishFanMode(); applyFanControl(); }
    // unknown payloads are ignored
  }
  else if (strcmp(topic, TOPIC_ON_C_SET) == 0) {
    float v = msg.toFloat();
    if (!(v > fanOffC)) v = fanOffC + 0.5f;   // enforce ON > OFF (0.5 °C gap)
    fanOnC = v;
    publishFanConfig();
    if (fanMode == FanMode::AUTO) applyFanControl();
  }
  else if (strcmp(topic, TOPIC_OFF_C_SET) == 0) {
    float v = msg.toFloat();
    if (!(v < fanOnC)) v = fanOnC - 0.5f;     // enforce OFF < ON (0.5 °C gap)
    fanOffC = v;
    publishFanConfig();
    if (fanMode == FanMode::AUTO) applyFanControl();
  }

  hasFreshData = true;
}

/**
 * @brief Ensure MQTT is connected; connect with LWT, subscribe, and republish retained state.
 */
void ensureMqtt() {
  while (!mqtt.connected()) {
    const String cid = "ESP32Display-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    // Connect with Last Will (broker publishes "offline" on unexpected drop)
    if (mqtt.connect(
          cid.c_str(),
          MQTT_USER, MQTT_PASSWORD,
          TOPIC_AVAIL,          // willTopic
          0,                    // willQoS
          true,                 // willRetain
          AVAIL_OFFLINE         // willMessage
        )) {

      // we are online now — publish retained "online"
      mqtt.publish(TOPIC_AVAIL, AVAIL_ONLINE, true);

      subscribeTopics();
      publishFanMode();
      publishFanConfig();
      publishFanState();
      publishDs18IfValid();

    } else {
      delay(2000);
    }
  }
}

// ============================================================================
// SECTION: DISPLAY (SSD1306 UI)
// ============================================================================

/**
 * @brief Draw the 3-line UI with two left-aligned columns.
 */
void drawScreen() {
  display.clearDisplay();
  display.setTextSize(1);

// --- Row 1: MQTT temp + humidity ---
  display.setCursor(COL1_X, ROW_Y0);
  display.print(haTemp); display.write((uint8_t)248); display.print("C"); // 248 = '°' with cp437
  display.setCursor(COL2_X, ROW_Y0);
  display.print(haHumi); display.print("%");

// --- Row 2: MQTT pressure + FAN state ---
  display.setCursor(COL1_X, ROW_Y1);
  display.print(haPress); display.print(" hPa");
  display.setCursor(COL2_X, ROW_Y1);
  display.print("FAN "); display.print(fanOn ? "ON" : "OFF");

// --- Row 3: local DS18B20 (integer) + LAST MQTT update time (HH:MM) ---
  display.setCursor(COL1_X, ROW_Y2);
  if (isValidDs18(dsTempC)) { display.print((int)round(dsTempC)); display.write((uint8_t)248); display.print("C"); }
  else                   { display.print("--C"); }
  display.setCursor(COL2_X, ROW_Y2);
  display.print(lastUpdateHHMM);

  display.display();
}

// ============================================================================
// SECTION: INIT HELPERS (Wi-Fi, MQTT)
// ============================================================================

/**
 * @brief Connect to Wi-Fi in STA mode (blocking until connected).
 */
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) { delay(250); }
}
/**
 * @brief Configure MQTT server and callback.
 */
void initMQTT() {
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
}

// ============================================================================
// SECTION: ARDUINO FRAMEWORK (ESP32): setup / loop
// ============================================================================

void setup() {
  Serial.begin(115200);

// --- I²C and OLED ---
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.cp437(true);         // enable Code Page 437 (for the '°' character at 248)
  display.setTextWrap(false);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();

// --- MOSFET gate ---
  pinMode(PIN_FAN, OUTPUT);
  digitalWrite(PIN_FAN, LOW);  // OFF at boot

// --- DS18B20 ---
  sensors.begin();

// --- Wi-Fi + time + MQTT ---
  initWiFi();
  timeClient.begin();
  timeClient.update();
  initMQTT();

// --- Initial draw ---
  lastUpdateHHMM = "--:--";
  hasFreshData = true;
  drawScreen();
}

void loop() {
  ensureMqtt();
  mqtt.loop();

// --- Keep NTP fresh (local time flows continuously between polls) ---
  timeClient.update();

// --- DS18B20 every DS18B20_SAMPLE_MS ---
  static uint32_t lastTempMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastTempMs >= DS18B20_SAMPLE_MS) {
    lastTempMs = nowMs;
    readDs18b20();
    hasFreshData = true;
  }

// --- Redraw when data changed, or as a safety every REDRAW_INTERVAL_MS ---
  static uint32_t lastRedrawMs = 0;
  if (hasFreshData || (nowMs - lastRedrawMs >= REDRAW_INTERVAL_MS)) {
    drawScreen();
    lastRedrawMs = nowMs;
    hasFreshData = false;
  }

  delay(10);
}
