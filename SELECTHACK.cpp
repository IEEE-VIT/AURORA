#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

/* ---------- WiFi ---------- */
const char* ssid = "Nord 3";
const char* password = "Nordpass";

/* ---------- MQTT ---------- */
const char* mqtt_server = "e1e9f66e.ala.asia-southeast1.emqxsl.com";
const int mqtt_port = 8883;
const char* mqtt_user = "esp_client";
const char* mqtt_pass = "Emqx@2006";
const char* mqtt_topic = "vision/zone1";

/* ---------- Hardware ---------- */
#define DHTPIN 21
#define DHTTYPE DHT11

#define RELAY_LED 26     // Channel 1
#define RELAY_FAN 27     // Channel 2

DHT dht(DHTPIN, DHTTYPE);

/* ---------- Control Params ---------- */
const float COMFORT_TEMP = 20.0;
const float TEMP_HYST = 1.0;
const int MIN_OCCUPANCY_ON = 2;
const unsigned long UNOCCUPIED_DELAY = 5UL * 60UL * 1000UL;

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
    peopleCount = doc["count"].as<int>();
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

  // No occupancy → delayed OFF
  if (peopleCount == 0) {
    if (millis() - lastOccupiedTime > UNOCCUPIED_DELAY) {
      desiredState = false;
    }
  } else {
    // Occupied logic with hysteresis
    if (temp < (COMFORT_TEMP - TEMP_HYST)) {
      desiredState = false;
    }
    else if (peopleCount >= MIN_OCCUPANCY_ON &&
             temp > (COMFORT_TEMP + TEMP_HYST)) {
      desiredState = true;
    }
    else if (peopleCount >= 1 && temp >= COMFORT_TEMP) {
      desiredState = true;
    }
  }

  // Apply relay change
  if (desiredState != relayState) {
    setRelay(desiredState);
  }

  // REQUIRED SERIAL FORMAT
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
  client.loop();      // MQTT keep-alive
  controlLogic();
  delay(1500);        // Safe for DHT11
}
