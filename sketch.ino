#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define PIN_LED_GREEN   25
#define PIN_LED_YELLOW  26
#define PIN_LED_RED     27
#define PIN_BUZZER      18
#define PIN_BUTTON      19

const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";
const char* MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* DEVICE_ID   = "device01";

const char* TOPIC_TELEMETRY = "careguard/device01/telemetry";
const char* TOPIC_STATUS    = "careguard/device01/status";
const char* TOPIC_EVENTS    = "careguard/device01/events";

const float FREEFALL_G_THRESHOLD = 0.45;
const float IMPACT_G_THRESHOLD   = 1.8;
const float ROTATION_DPS_THRESH  = 15.0;
const unsigned long CONFIRM_DELAY_MS = 4000;

enum SystemState {
  STATE_NORMAL,
  STATE_SUSPECTED_FALL,
  STATE_FALL
};

SystemState currentState = STATE_NORMAL;
unsigned long suspectedStartTime = 0;
unsigned long lastSampleTime = 0;
unsigned long lastTelemetryTime = 0;

Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setIndicators(bool g, bool y, bool r, bool buzzerTone) {
  digitalWrite(PIN_LED_GREEN, g ? HIGH : LOW);
  digitalWrite(PIN_LED_YELLOW, y ? HIGH : LOW);
  digitalWrite(PIN_LED_RED, r ? HIGH : LOW);
  if (buzzerTone) {
    tone(PIN_BUZZER, 1000);
  } else {
    noTone(PIN_BUZZER);
  }
}

void publishEvent(const char* eventType, const char* detail) {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<256> doc;
  doc["device_id"] = DEVICE_ID;
  doc["timestamp_ms"] = millis();
  doc["event"] = eventType;
  doc["detail"] = detail;

  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(TOPIC_EVENTS, buffer);
}

void publishStatus(const char* stateStr) {
  if (!mqttClient.connected()) return;
  StaticJsonDocument<200> doc;
  doc["device_id"] = DEVICE_ID;
  doc["timestamp_ms"] = millis();
  doc["status"] = stateStr;

  char buffer[200];
  serializeJson(doc, buffer);
  mqttClient.publish(TOPIC_STATUS, buffer, true);
}

void connectNetwork() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(300);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  } else {
    Serial.println("\nWiFi Offline");
  }
}

void checkMqttConnection() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "CareGuard-Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("connected!");
      publishStatus("NORMAL");
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  setIndicators(true, false, false, false);

  if (!mpu.begin()) {
    Serial.println("MPU6050 tidak ditemukan");
    while (1) {
      digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
      delay(200);
    }
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectNetwork();
}

void loop() {
  mqttClient.loop();
  checkMqttConnection();

  static int lastBtnState = HIGH;
  int btnState = digitalRead(PIN_BUTTON);
  if (lastBtnState == HIGH && btnState == LOW) {
    delay(50);
    if (currentState == STATE_SUSPECTED_FALL) {
      Serial.println("[USER] Alarm dibatalkan.");
      currentState = STATE_NORMAL;
      publishEvent("CANCEL", "False alarm cancelled by user");
      publishStatus("NORMAL");
    } else if (currentState == STATE_NORMAL) {
      Serial.println("[USER] SOS aktif.");
      currentState = STATE_FALL;
      publishEvent("SOS", "Manual SOS triggered");
      publishStatus("FALL");
    } else if (currentState == STATE_FALL) {
      Serial.println("[USER] Reset sistem.");
      currentState = STATE_NORMAL;
      publishEvent("RESET", "Fall state reset by user");
      publishStatus("NORMAL");
    }
  }
  lastBtnState = btnState;

  if (millis() - lastSampleTime >= 20) {
    lastSampleTime = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax_g = a.acceleration.x / 9.80665;
    float ay_g = a.acceleration.y / 9.80665;
    float az_g = a.acceleration.z / 9.80665;
    float accel_mag = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

    float gx_dps = g.gyro.x * 57.2958;
    float gy_dps = g.gyro.y * 57.2958;
    float gz_dps = g.gyro.z * 57.2958;
    float gyro_mag = sqrt(gx_dps * gx_dps + gy_dps * gy_dps + gz_dps * gz_dps);

    if (currentState == STATE_NORMAL) {
      bool freefall = (accel_mag < FREEFALL_G_THRESHOLD);
      bool impact   = (accel_mag > IMPACT_G_THRESHOLD);
      bool rotation = (gyro_mag > ROTATION_DPS_THRESH);

      if (freefall || impact || rotation) {
        Serial.printf("[DETEKSI] Suspected Fall! Mag: %.2f g, Gyro: %.2f dps\n", accel_mag, gyro_mag);
        currentState = STATE_SUSPECTED_FALL;
        suspectedStartTime = millis();
        publishEvent("SUSPECTED_FALL", "Abnormal movement detected");
        publishStatus("SUSPECTED_FALL");
      }
    } 
    else if (currentState == STATE_SUSPECTED_FALL) {
      if (millis() - suspectedStartTime >= CONFIRM_DELAY_MS) {
        Serial.println("[ALERT] Konfirmasi jatuh!");
        currentState = STATE_FALL;
        publishEvent("FALL", "Fall confirmed");
        publishStatus("FALL");
      }
    }

    if (millis() - lastTelemetryTime >= 500) {
      lastTelemetryTime = millis();
      if (mqttClient.connected()) {
        StaticJsonDocument<256> doc;
        doc["device_id"] = DEVICE_ID;
        doc["timestamp_ms"] = millis();
        doc["ax"] = serialized(String(ax_g, 2));
        doc["ay"] = serialized(String(ay_g, 2));
        doc["az"] = serialized(String(az_g, 2));
        doc["gx"] = serialized(String(gx_dps, 1));
        doc["gy"] = serialized(String(gy_dps, 1));
        doc["gz"] = serialized(String(gz_dps, 1));
        doc["magnitude"] = serialized(String(accel_mag, 2));
        doc["status"] = (currentState == STATE_NORMAL) ? "NORMAL" :
                        (currentState == STATE_SUSPECTED_FALL) ? "SUSPECTED_FALL" : "FALL";

        char buffer[256];
        serializeJson(doc, buffer);
        mqttClient.publish(TOPIC_TELEMETRY, buffer);
      }
    }
  }

  switch (currentState) {
    case STATE_NORMAL:
      setIndicators(true, false, false, false);
      break;
    case STATE_SUSPECTED_FALL:
      setIndicators(false, (millis() / 250) % 2 == 0, false, (millis() / 250) % 2 == 0);
      break;
    case STATE_FALL:
      setIndicators(false, false, true, true);
      break;
  }
}