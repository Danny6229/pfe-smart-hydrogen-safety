// 1. Include Required Libraries
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h> 
#include "DHT.h"

// 2. Wi-Fi Settings
const char* ssid     = "MyTestNet"; 
const char* pass     = "123456789";

// 3. MQTT Broker Settings
const char* mqtt_server = "10.57.250.153"; 
const int mqtt_port     = 1883;

// 4. Pin Assignments
#define DHTPIN 17       // DHT22 Data
#define FANPIN 18       // Relay Control Fan
#define RED_LED 25      // RGB LED Red Pin
#define GREEN_LED 26    // RGB LED Green Pin
#define BLUE_LED 27     // RGB LED Blue Pin
#define BUZZER 33       // Safety Alarm Buzzer Module 'S' Pin
#define CURRENT_PIN 34  // Analog Current Sensor Pin (Out)
#define VOLTAGE_PIN 35  // Analog Voltage Sensor Pin (S)
#define MQ8_PIN 32      // MQ-8 Hydrogen Sensor Analog Pin
#define DHTTYPE DHT22 

// 5. Object Initialization
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// Control Logic Variables
float baselineTemp = 0.0;
bool baselineEstablished = false;
const float TEMP_THRESHOLD = 1.0;

// Global tracking flags for Hysteresis loop matching
int fanState = 0;
int buzzerState = 0;
int criticalAlarm = 0;

// Calibration Tuning Parameters
const float SENSITIVITY = 185.0;  // 185 mV per Amp for 5A ACS712 variant
float zeroCurrentVoltage = 2.345; // Dynamically updated on boot

// Tracking parameters for loops
unsigned long lastMsgTime = 0;
const long interval = 2000; // Sample and broadcast every 2 seconds

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected successfully!");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to local MQTT Broker...");
    String clientId = "ESP32_Safety_Stack-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("CONNECTED!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  dht.begin();
  pinMode(FANPIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(MQ8_PIN, INPUT); 
  
  analogSetAttenuation(ADC_11db);

  // Initialize hardware states
  digitalWrite(FANPIN, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH); 
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER, LOW);

  // Read raw sensor resting voltage state before anything triggers for absolute auto-calibration
  long initialSum = 0;
  for (int i = 0; i < 100; i++) {
    initialSum += analogRead(CURRENT_PIN);
    delay(5);
  }
  zeroCurrentVoltage = (initialSum / 100.0 / 4095.0) * 3.3;
  Serial.print(F("Auto-calibrated Zero Current Base Point: "));
  Serial.print(zeroCurrentVoltage, 3);
  Serial.println(F(" V"));

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsgTime > interval) {
    lastMsgTime = now;

    // --- 1. Environmental Readings ---
    float currentTemp = dht.readTemperature();
    float currentHumidity = dht.readHumidity();

    if (isnan(currentTemp) || isnan(currentHumidity)) {
      Serial.println(F("Error: Failed to read from DHT sensor!"));
      return;
    }

    if (!baselineEstablished) {
      baselineTemp = currentTemp;
      baselineEstablished = true;
      Serial.print(F("Baseline Temperature established: "));
      Serial.println(baselineTemp);
    }

    float temperatureRise = currentTemp - baselineTemp;

    // --- 2. Live Calibrated Hardware Calculations ---
    long voltageSum = 0;
    for (int i = 0; i < 50; i++) {
      voltageSum += analogRead(VOLTAGE_PIN);
      delayMicroseconds(200);
    }
    float calculatedVoltage = (voltageSum / 50.0 / 4095.0) * 3.3 * 5.0;

    long currentSum = 0;
    for (int i = 0; i < 50; i++) {
      currentSum += analogRead(CURRENT_PIN);
      delayMicroseconds(200);
    }
    float currentPinVolts = (currentSum / 50.0 / 4095.0) * 3.3;
    
    float raw_mA = ((currentPinVolts - zeroCurrentVoltage) / (SENSITIVITY / 1000.0)) * 1000.0;
    float current_mA = abs(raw_mA);

    // --- 3. MQ-8 Hydrogen Gas Calculations (Raw & Percentage) ---
    long hydrogenSum = 0;
    for (int i = 0; i < 20; i++) {
      hydrogenSum += analogRead(MQ8_PIN);
      delayMicroseconds(100);
    }
    float rawHydrogen = hydrogenSum / 20.0; 

    if (rawHydrogen < 10.0) rawHydrogen = 10.0; 
    
    // Map the relative drop in resistance to a 0% to 100% hazard scale
    float cleanAirBaseline = 2500.0; 
    float hydrogenPercentage = 0.0;
    
    if (rawHydrogen > cleanAirBaseline) {
      hydrogenPercentage = ((rawHydrogen - cleanAirBaseline) / (4095.0 - cleanAirBaseline)) * 100.0;
    } else {
      hydrogenPercentage = 0.0;
    }

    // --- 4. Safety Threshold Check & Local Actuator Control (Hysteresis Loop) ---
    
    // CONDITION A: Crosses the critical 15% threshold or experiences temperature spike
    if (hydrogenPercentage >= 15.0 || temperatureRise >= TEMP_THRESHOLD) {
      digitalWrite(FANPIN, HIGH);
      digitalWrite(RED_LED, HIGH);   
      digitalWrite(GREEN_LED, LOW);  
      digitalWrite(BLUE_LED, LOW);   
      
      // Hardware passive piezo alarm executes while threshold is breached
      for (int i = 0; i < 100; i++) { 
        digitalWrite(BUZZER, HIGH); delayMicroseconds(250); 
        digitalWrite(BUZZER, LOW);  delayMicroseconds(250); 
      }
      fanState = 1; 
      buzzerState = 1;
      criticalAlarm = 1;
    } 
    // CONDITION B: Clearing Zone (Hydrogen dropped below 15% but still remains above or equal to 4%)
    else if (hydrogenPercentage >= 4.0 && fanState == 1) {
      digitalWrite(FANPIN, HIGH);    // Keep the ventilation fan latched ON
      digitalWrite(RED_LED, LOW);     
      digitalWrite(GREEN_LED, LOW);  
      digitalWrite(BLUE_LED, HIGH);   // Indicator switches to Blue to show active venting
      digitalWrite(BUZZER, LOW);      // Buzzer turns OFF safely as requested
      
      fanState = 1; 
      buzzerState = 0;
      criticalAlarm = 0;              // Clears primary alarm indicator for Grafana safety block
    } 
    // CONDITION C: Completely Safe Zone (Hydrogen drops safely below 4%)
    else {
      digitalWrite(FANPIN, LOW);     // Shut down fan safely
      digitalWrite(RED_LED, LOW);     
      digitalWrite(GREEN_LED, HIGH);  // Back to Green idle system status
      digitalWrite(BLUE_LED, LOW);    
      digitalWrite(BUZZER, LOW);      // Ensure buzzer stays silent
      
      fanState = 0; 
      buzzerState = 0;
      criticalAlarm = 0;
    }

    // --- 5. Print Data to Serial Monitor ---
    Serial.print(F("Temp: ")); Serial.print(currentTemp); Serial.print(F("°C | "));
    Serial.print(F("Rise: ")); Serial.print(temperatureRise, 1); Serial.print(F("°C | "));
    Serial.print(F("Hydrogen (Raw): ")); Serial.print(rawHydrogen, 0); Serial.print(F(" | "));
    Serial.print(F("Hydrogen (%): ")); Serial.print(hydrogenPercentage, 2); Serial.print(F("% | "));
    Serial.print(F("Voltage: ")); Serial.print(calculatedVoltage, 2); Serial.print(F(" V | "));
    Serial.print(F("Current: ")); Serial.print(current_mA, 1); Serial.println(F(" mA"));

    // --- 6. JSON Compilation & MQTT Publish ---
    StaticJsonDocument<256> doc;
    
    doc["hydrogen_raw"]   = rawHydrogen; 
    doc["hydrogen_pct"]   = hydrogenPercentage;
    doc["temperature"]    = currentTemp;
    doc["humidity"]       = currentHumidity;
    doc["voltage"]        = calculatedVoltage;
    
    // Converts mA back to Amperes (A) for Grafana unit parsing
    doc["current"]        = current_mA / 1000.0; 
    
    doc["fan_status"]     = fanState;
    doc["buzzer_status"]  = buzzerState;
    doc["critical_alarm"] = criticalAlarm;

    char jsonBuffer[256];
    serializeJson(doc, jsonBuffer);
    
    client.publish("esp32/telemetry", jsonBuffer);
  }
}
