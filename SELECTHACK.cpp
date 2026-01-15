#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#include "config.h"

/* ---------- WiFi ---------- */
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

/* ---------- MQTT ---------- */
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USERNAME;
const char* mqtt_pass = MQTT_PASSWORD;
const char* mqtt_topic = MQTT_TOPIC;

/* ---------- Hardware ---------- */
#define DHTPIN     DHT_PIN
#define DHTTYPE    DHT_SENSOR_TYPE

#define RELAY_LED  RELAY_LED_PIN
#define RELAY_FAN  RELAY_FAN_PIN

DHT dht(DHTPIN, DHTTYPE);

/* ---------- Control Params ---------- */
const float COMFORT_TEMP = COMFORT_TEMP_C;
const float TEMP_HYST = TEMP_HYST_C;
const int MIN_OCCUPANCY = MIN_OCCUPANCY_ON;
const unsigned long UNOCCUPIED_DELAY = UNOCCUPIED_DELAY_MS;

/* ---------- State ---------- */
volatile int peopleCount = 0;
unsigned long lastOccupiedTime = 0;
bool relayState = false;

/* ---------- MQTT Client ---------- */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* ---------- Relay Control ---------- */
void setRelay(bool state) {
  relayState = state;

  // LOW-trigger relay
  digitalWrite(RELAY_LED, state ? LOW : HIGH);
  digitalWrite(RELAY_FAN, state ? LOW : HIGH);
}

/* ---------- MQTT Callback ---------- */
void callback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<512> doc;

  if (deserializeJson(doc, payload, length)) {
    Serial.println("JSON parse error");
    return;
  }

  if (doc["count"].is<int>()) {
    peopleCount = doc["count"];
  } else if (doc["count"].is<const char*>()) {
    peopleCount = atoi(doc["count"]);
  } else {
    Serial.println("Invalid count type");
    return;
  }

  if (peopleCount > 0) {
    lastOccupiedTime = millis();
  }
}

/* ---------- Control Logic ---------- */
void controlLogic() {
  float temp = dht.readTemperature();
  if (isnan(temp)) return;

  bool desiredState = relayState;

  // Unoccupied → delayed OFF
  if (peopleCount == 0) {
    if (millis() - lastOccupiedTime > UNOCCUPIED_DELAY) {
      desiredState = false;
    }
  } 
  else {
    // Occupied logic with hysteresis
    if (temp < (COMFORT_TEMP - TEMP_HYST)) {
      desiredState = false;
    }
    else if (peopleCount >= MIN_OCCUPANCY &&
             temp > (COMFORT_TEMP + TEMP_HYST)) {
      desiredState = true;
    }
    else if (peopleCount >= 1 && temp >= COMFORT_TEMP) {
      desiredState = true;
    }
  }

  if (desiredState != relayState) {
    setRelay(desiredState);
  }

  // Required serial format
  Serial.print("People=");
  Serial.print(peopleCount);
  Serial.print(" | Temp=");
  Serial.print(temp, 1);
  Serial.print(" | LED=");
  Serial.print(relayState ? "ON" : "OFF");
  Serial.print(" | FAN=");
  Serial.println(relayState ? "ON" : "OFF");
}

/* ---------- Setup ---------- */
void setup() {
  Serial.begin(115200);

  pinMode(RELAY_LED, OUTPUT);
  pinMode(RELAY_FAN, OUTPUT);

  // Relay OFF initially (LOW-trigger)
  digitalWrite(RELAY_LED, HIGH);
  digitalWrite(RELAY_FAN, HIGH);

  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  while (!client.connected()) {
    if (client.connect("ESP32-energy", mqtt_user, mqtt_pass)) {
      client.subscribe(mqtt_topic);
    } else {
      delay(2000);
    }
  }
}

/* ---------- Loop ---------- */
void loop() {
  client.loop();
  controlLogic();
  delay(1500);  // Safe for DHT11
}
