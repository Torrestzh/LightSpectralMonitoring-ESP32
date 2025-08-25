#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_AS7341.h>
#include <Adafruit_TSL2591.h>
#include <WiFi.h>
#include <PubSubClient.h>

//  OLED 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//  LEDs 
#define RED_LED     14
#define YELLOW_LED  27
#define GREEN_LED   26

//  WiFi & MQTT 
const char* ssid = "Torres";
const char* password = "13028500862tzh";
const char* mqtt_server = "broker.hivemq.com";

// Topic: Real time and state separation
const char* MQTT_TOPIC_LIVE   = "esp32/light_sensor_data/live";    // Real time streaming (without reservation)
const char* MQTT_TOPIC_STATUS = "esp32/light_sensor_data/status";  // Online and offline status (reserved)

WiFiClient espClient;
PubSubClient client(espClient);

// Sensors 
Adafruit_AS7341 as7341;
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);

// Parameter recording (for serial port&MQTT reporting) 
const char* TSL_time_str = "100ms";
const char* TSL_gain_str = "LOW";
const char* AS_gain_str  = "4X";
uint8_t  AS_atime = 100;
uint16_t AS_astep = 200;

// Data caching (synchronous display/reporting)
uint16_t cached_full = 0, cached_ir = 0, cached_visible = 0;
float    cached_lux = 0.0f;
uint16_t cached_f[8] = {0};
uint16_t cached_clear = 0, cached_nir = 0;

// utility function
static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWiFi connected" : "\nWiFi failed");
}

bool mqttConnect() {
  // Unique Client ID to avoid conflicts with PC
  String cid = "ESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

  // LWT: When offline, the broker automatically publishes to the status topic
  const char* willMsg = "{\"status\":\"offline\"}";
  bool ok = client.connect(cid.c_str(), MQTT_TOPIC_STATUS, 0, true, willMsg);
  if (ok) {
    // Online status
    client.publish(MQTT_TOPIC_STATUS, "{\"status\":\"online\"}", true);
  }
  return ok;
}

void reconnect_mqtt() {
  if (!client.connected() && WiFi.status() == WL_CONNECTED) {
    if (mqttConnect()) {
      Serial.println("MQTT connected");
    } else {
      Serial.println("MQTT connect failed");
    }
  }
}

// Automatic range: TSL2591 (multiple levels+anti flicker)
void autoAdjustTSL2591() {
  const uint16_t TARGET_LOW = 8000, TARGET_HIGH = 45000;
  const tsl2591IntegrationTime_t times[] = {
    TSL2591_INTEGRATIONTIME_100MS, TSL2591_INTEGRATIONTIME_200MS,
    TSL2591_INTEGRATIONTIME_300MS, TSL2591_INTEGRATIONTIME_400MS
  };
  const char* timeStr[] = {"100ms","200ms","300ms","400ms"};
  const tsl2591Gain_t gains[] = {
    TSL2591_GAIN_LOW, TSL2591_GAIN_MED, TSL2591_GAIN_HIGH
  };
  const char* gainStr[] = {"LOW","MED","HIGH"};

  for (int ti = 0; ti < 4; ++ti) {
    tsl2591.setTiming(times[ti]);
    for (int gi = 0; gi < 3; ++gi) {
      tsl2591.setGain(gains[gi]);
      delay(2);
      (void)tsl2591.getFullLuminosity();
      uint16_t fullArr[3];
      for (int k = 0; k < 3; ++k) {
        uint32_t lum = tsl2591.getFullLuminosity();
        fullArr[k] = lum & 0xFFFF;
        delay(5);
      }
      uint16_t fullMed = median3(fullArr[0], fullArr[1], fullArr[2]);
      if (fullMed >= TARGET_LOW && fullMed <= TARGET_HIGH) {
        TSL_time_str = timeStr[ti];
        TSL_gain_str = gainStr[gi];
        return;
      }
    }
  }
  TSL_time_str = timeStr[3];
  TSL_gain_str = gainStr[2];
}

// Automatic range: AS7341 (multiple levels+anti flicker)
uint16_t as7341ReadMaxOnce() {
  if (!as7341.readAllChannels()) return 0;
  uint16_t arr[10] = {
    as7341.getChannel(AS7341_CHANNEL_415nm_F1),
    as7341.getChannel(AS7341_CHANNEL_445nm_F2),
    as7341.getChannel(AS7341_CHANNEL_480nm_F3),
    as7341.getChannel(AS7341_CHANNEL_515nm_F4),
    as7341.getChannel(AS7341_CHANNEL_555nm_F5),
    as7341.getChannel(AS7341_CHANNEL_590nm_F6),
    as7341.getChannel(AS7341_CHANNEL_630nm_F7),
    as7341.getChannel(AS7341_CHANNEL_680nm_F8),
    as7341.getChannel(AS7341_CHANNEL_CLEAR),
    as7341.getChannel(AS7341_CHANNEL_NIR)
  };
  uint16_t m = 0;
  for (int i = 0; i < 10; i++) if (arr[i] > m) m = arr[i];
  return m;
}

uint16_t as7341ReadMaxMedian3() {
  (void)as7341ReadMaxOnce();
  uint16_t m1 = as7341ReadMaxOnce(); delay(5);
  uint16_t m2 = as7341ReadMaxOnce(); delay(5);
  uint16_t m3 = as7341ReadMaxOnce(); delay(5);
  return median3(m1, m2, m3);
}

void autoAdjustAS7341() {
  const uint16_t TARGET_LOW = 8000, TARGET_HIGH = 45000;
  const uint16_t astepList[] = {100, 300, 600, 900};
  const as7341_gain_t gainList[] = {AS7341_GAIN_1X, AS7341_GAIN_4X, AS7341_GAIN_16X};
  const char* gainStr[] = {"1X","4X","16X"};

  as7341.setATIME(AS_atime);
  for (int ai = 0; ai < 4; ++ai) {
    as7341.setASTEP(astepList[ai]);
    for (int gi = 0; gi < 3; ++gi) {
      as7341.setGain(gainList[gi]);
      delay(2);
      uint16_t maxMed = as7341ReadMaxMedian3();
      if (maxMed >= TARGET_LOW && maxMed <= TARGET_HIGH) {
        AS_astep = astepList[ai];
        AS_gain_str = gainStr[gi];
        return;
      }
    }
  }
  AS_astep = astepList[3];
  AS_gain_str = gainStr[2];
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(RED_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed"); while (1);
  }
  display.clearDisplay();

  if (!as7341.begin()) { Serial.println("AS7341 init failed"); while (1); }
  if (!tsl2591.begin()) { Serial.println("TSL2591 init failed"); while (1); }

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setBufferSize(2048); 
  client.setKeepAlive(30);   
}

void loop() {
  if (!client.connected()) reconnect_mqtt();
  client.loop();

  // Synchronize the collection of two sensors (write to cache)
  autoAdjustTSL2591(); client.loop();
  autoAdjustAS7341(); client.loop();

  // TSL2591 
  uint32_t lum_once = tsl2591.getFullLuminosity();
  cached_ir   = (uint16_t)(lum_once >> 16);
  cached_full = (uint16_t)(lum_once & 0xFFFF);
  cached_visible = (cached_full > cached_ir) ? (cached_full - cached_ir) : 0;
  sensors_event_t event;
  tsl2591.getEvent(&event);
  cached_lux = event.light;

  // AS7341 
  as7341.readAllChannels();
  cached_f[0] = as7341.getChannel(AS7341_CHANNEL_415nm_F1);
  cached_f[1] = as7341.getChannel(AS7341_CHANNEL_445nm_F2);
  cached_f[2] = as7341.getChannel(AS7341_CHANNEL_480nm_F3);
  cached_f[3] = as7341.getChannel(AS7341_CHANNEL_515nm_F4);
  cached_f[4] = as7341.getChannel(AS7341_CHANNEL_555nm_F5);
  cached_f[5] = as7341.getChannel(AS7341_CHANNEL_590nm_F6);
  cached_f[6] = as7341.getChannel(AS7341_CHANNEL_630nm_F7);
  cached_f[7] = as7341.getChannel(AS7341_CHANNEL_680nm_F8);
  cached_clear = as7341.getChannel(AS7341_CHANNEL_CLEAR);
  cached_nir   = as7341.getChannel(AS7341_CHANNEL_NIR);

  // LED indicator
  const char* statusText;
  if (cached_lux < 1000) {
    digitalWrite(RED_LED, LOW); digitalWrite(YELLOW_LED, HIGH); digitalWrite(GREEN_LED, LOW);
    statusText = "LOW";
  } else if (cached_lux < 10000) {
    digitalWrite(RED_LED, LOW); digitalWrite(YELLOW_LED, LOW); digitalWrite(GREEN_LED, HIGH);
    statusText = "MED";
  } else {
    digitalWrite(RED_LED, HIGH); digitalWrite(YELLOW_LED, LOW); digitalWrite(GREEN_LED, LOW);
    statusText = "HIGH";
  }

  // OLED single page display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setTextWrap(false);

  char line[26];
  display.setCursor(0, 0);
  snprintf(line, sizeof(line), "Lux:%5.1f  S:%s", cached_lux, statusText);
  display.println(line);

  display.setCursor(0, 8);
  snprintf(line, sizeof(line), "Full:%5u  IR:%5u", cached_full, cached_ir);
  display.println(line);

  display.setCursor(0, 16);
  snprintf(line, sizeof(line), "Vis:%5u", cached_visible);   
  display.println(line);

  display.setCursor(0, 24);
  snprintf(line, sizeof(line), "NIR:%5u  Clr:%5u", cached_nir, cached_clear); 
  display.println(line);

  display.setCursor(0, 32);
  snprintf(line, sizeof(line), "F1:%4u  F5:%4u", cached_f[0], cached_f[4]);
  display.println(line);

  display.setCursor(0, 40);
  snprintf(line, sizeof(line), "F2:%4u  F6:%4u", cached_f[1], cached_f[5]);
  display.println(line);

  display.setCursor(0, 48);
  snprintf(line, sizeof(line), "F3:%4u  F7:%4u", cached_f[2], cached_f[6]);
  display.println(line);

  display.setCursor(0, 56);
  snprintf(line, sizeof(line), "F4:%4u  F8:%4u", cached_f[3], cached_f[7]);
  display.println(line);

  display.display();

  // serial port output
  Serial.println("=== TSL2591 ===");
  Serial.printf("Full:%u IR:%u Vis:%u Lux:%.1f  t:%s g:%s\n",
                cached_full, cached_ir, cached_visible, cached_lux,
                TSL_time_str, TSL_gain_str);

  Serial.println("=== AS7341 ===");
  for (int i = 0; i < 8; i++) {
    Serial.printf("F%d:%u ", i+1, cached_f[i]);
  }
  Serial.printf("Clr:%u NIR:%u  AT:%u AS:%u G:%s\n",
                cached_clear, cached_nir, AS_atime, AS_astep, AS_gain_str);

  // MQTT upload
  if (client.connected()) {
    char payload[640];
    snprintf(payload, sizeof(payload),
      "{"
        "\"full\":%u,\"ir\":%u,\"visible\":%u,\"lux\":%.1f,"
        "\"f1\":%u,\"f2\":%u,\"f3\":%u,\"f4\":%u,\"f5\":%u,\"f6\":%u,\"f7\":%u,\"f8\":%u,"
        "\"clear\":%u,\"nir\":%u,"
        "\"status\":\"%s\","
        "\"tsl_time\":\"%s\",\"tsl_gain\":\"%s\","
        "\"as_atime\":%u,\"as_astep\":%u,\"as_gain\":\"%s\""
      "}",
      cached_full, cached_ir, cached_visible, cached_lux,
      cached_f[0], cached_f[1], cached_f[2], cached_f[3], cached_f[4], cached_f[5], cached_f[6], cached_f[7],
      cached_clear, cached_nir,
      statusText,
      TSL_time_str, TSL_gain_str,
      AS_atime, AS_astep, AS_gain_str
    );
    bool ok = client.publish(MQTT_TOPIC_LIVE, payload, false); 
    Serial.println(ok ? "MQTT publish OK" : "MQTT publish FAIL");
  }

  client.loop();

  for (int i = 0; i < 20; ++i) { delay(100); client.loop(); }
}
