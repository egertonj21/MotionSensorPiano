#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

// Constants for configuration
const char* MQTT_SERVER = "server IP goes here";
const int MQTT_PORT = 1883;
const char* CONTROL_TOPIC = "control/distance_sensor";
const char* MOTION_CONTROL_TOPIC = "control/motion_sensor"; // Topic for motion sensor control
const char* HEARTBEAT_TOPIC = "heartbeat/motion_sensor"; // Topic for heartbeat messages

// PIR Sensor pin
const int PIR_PIN = 10;

// Variables
WiFiClient espClient;
PubSubClient client(espClient);
volatile bool motionDetected = false;
bool isAwake = true;
bool motionSensorActive = true;
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 60000; // 60 seconds

void detectMotion();
void sendMQTTMessage(const char* message, const char* topic = CONTROL_TOPIC);
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();

void setup() {
  Serial.begin(115200);

  // Initialize PIR Sensor pin
  pinMode(PIR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIR_PIN), detectMotion, RISING);

  // Setup WiFi
  WiFiManager wifiManager;
  wifiManager.autoConnect("PIR_Sensor");

  Serial.println("Connected to WiFi!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(callback);

  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Handle motion detection
  if (motionSensorActive && motionDetected) {
    Serial.println("Motion Detected!");
    sendMQTTMessage("wake");
    motionDetected = false;
    isAwake = true;
  }

  // Send heartbeat message periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    sendMQTTMessage("alive", HEARTBEAT_TOPIC);
    lastHeartbeatTime = currentMillis;
  }
}

void detectMotion() {
  if (motionSensorActive) {
    motionDetected = true;
    Serial.println("Motion detected - interrupt triggered");
  }
}

void sendMQTTMessage(const char* message, const char* topic) {
  Serial.print("Attempting to send MQTT message to ");
  Serial.print(topic);
  Serial.print(": ");
  Serial.println(message);
  if (client.publish(topic, message)) {
    Serial.print("MQTT Message Sent: ");
    Serial.println(message);
  } else {
    Serial.print("MQTT Message Failed: ");
    Serial.println(message);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String message = String((char*)payload);

  Serial.print("Received message on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(message);

  if (String(topic) == MOTION_CONTROL_TOPIC) {
    if (message == "sleep") {
      isAwake = false;
      motionSensorActive = false;
      Serial.println("Sensor is going to sleep...");
    } else if (message == "wake") {
      isAwake = true;
      motionSensorActive = true;
      Serial.println("Sensor is waking up...");
    }
  } else if (String(topic) == MOTION_CONTROL_TOPIC) {
    if (message == "motion_sleep") {
      motionSensorActive = false;
      Serial.println("Motion Sensor is deactivated...");
    } else if (message == "motion_wake") {
      motionSensorActive = true;
      Serial.println("Motion Sensor is activated...");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32PIRClient")) {
      Serial.println("connected");
      client.subscribe(CONTROL_TOPIC);
      client.subscribe(MOTION_CONTROL_TOPIC);
      Serial.print("Subscribed to topics: ");
      Serial.print(CONTROL_TOPIC);
      Serial.print(", ");
      Serial.println(MOTION_CONTROL_TOPIC);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
