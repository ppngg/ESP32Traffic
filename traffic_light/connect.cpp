#include "connect.h"
#include "secrects.h"
#include "traffic_light.h"   // <-- ADD THIS

#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
//YOUR AWS IOT TOPIC
#define AWS_IOT_TOPIC "esp32/traffic/density_now"

static WiFiClientSecure net;
static MQTTClient client(256);

// helper: read int from JSON even if value is "80" (string) or 80 (number)
static int readJsonInt(const JsonDocument& doc, const char* key, int defVal) {
  if (!doc.containsKey(key)) return defVal;

  if (doc[key].is<int>()) return doc[key].as<int>();

  if (doc[key].is<const char*>()) {
    const char* s = doc[key].as<const char*>();
    if (!s) return defVal;
    return atoi(s);
  }
  return defVal;
}

// helper: read float from JSON and convert to percentage (0-100)
// Handles both integer percentages (0-100) and decimal values (0.0-1.0)
static int readJsonPercentage(const JsonDocument& doc, const char* key, int defVal) {
  if (!doc.containsKey(key)) return defVal;

  float floatVal = 0.0;
  
  // Try to read as float (handles both int and float JSON numbers)
  if (doc[key].is<float>() || doc[key].is<double>() || doc[key].is<int>()) {
    floatVal = doc[key].as<float>();
  } else if (doc[key].is<const char*>()) {
    const char* s = doc[key].as<const char*>();
    if (!s) return defVal;
    floatVal = atof(s);
  } else {
    return defVal;
  }

  // If value is between 0.0 and 1.0, it's a decimal percentage (convert to 0-100)
  if (floatVal >= 0.0 && floatVal <= 1.0) {
    return (int)(floatVal * 100.0 + 0.5); // Round to nearest integer
  }
  // Otherwise, assume it's already a percentage (0-100)
  else if (floatVal >= 0.0 && floatVal <= 100.0) {
    return (int)(floatVal + 0.5); // Round to nearest integer
  }
  
  return defVal; // Invalid range
}

static void messageHandler(String &topic, String &payload) {
  Serial.println("=== MQTT MESSAGE ===");
  Serial.print("Topic: "); Serial.println(topic);
  Serial.print("Raw payload: "); Serial.println(payload);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    Serial.println("====================");
    return;
  }

  // optional sender field (if you include deviceId)
  const char* sender = doc["deviceId"] | "";
  Serial.print("Sender: ");
  Serial.println(sender[0] ? sender : "(none)");

  // ---- Accept density percentage values (0-100 or 0.0-1.0):
  // 1) {"density_now":80} or {"density_now":0.80} -> applies to both directions (80% of max vehicles)
  // 2) {"density_now_dir1":60,"density_now_dir2":80} -> separate directions
  // Handles both integer percentages (0-100) and decimal values (0.0-1.0 converted to 0-100)
  int density_percentage  = readJsonPercentage(doc, "density_now", -1);
  int d1_percentage = readJsonPercentage(doc, "density_now_dir1", -1);
  int d2_percentage = readJsonPercentage(doc, "density_now_dir2", -1);

  // Configuration: 100% density = 50 vehicles (max capacity)
  const int MAX_VEHICLES = 219;
  const int MIN_PERCENTAGE_THRESHOLD = 20; // Minimum percentage to update (0 = accept all valid values)

  // Convert percentage (0-100) to real vehicle count
  // Formula: real_density = (percentage * MAX_VEHICLES) / 100
  
  // If "density_now" exists: apply to both directions
  if (density_percentage >= MIN_PERCENTAGE_THRESHOLD && density_percentage <= 100) {
    int d = (density_percentage * MAX_VEHICLES) / 100;
    setDensityDir1(d);
    setDensityDir2(d);

    int g1 = compute_green_seconds(d);
    int g2 = compute_green_seconds(d);

    Serial.print("Parsed density_percentage = "); Serial.println(density_percentage);
    Serial.print("Converted to vehicles = "); Serial.println(d);
    Serial.print("Computed GREEN1 seconds = "); Serial.println(g1);
    Serial.print("Computed GREEN2 seconds = "); Serial.println(g2);
  }
  // Else if separate densities exist
  else if ((d1_percentage >= MIN_PERCENTAGE_THRESHOLD && d1_percentage <= 100) || 
           (d2_percentage >= MIN_PERCENTAGE_THRESHOLD && d2_percentage <= 100)) {
    int d1 = -1, d2 = -1;
    
    if (d1_percentage >= MIN_PERCENTAGE_THRESHOLD && d1_percentage <= 100) {
      d1 = (d1_percentage * MAX_VEHICLES) / 100;
      setDensityDir1(d1);
      int g1 = compute_green_seconds(d1);
      Serial.print("Parsed density_now_dir1 = "); Serial.print(d1_percentage);
      Serial.print("% -> vehicles = "); Serial.print(d1);
      Serial.print(" -> GREEN1 seconds = "); Serial.println(g1);
    }
    
    if (d2_percentage >= MIN_PERCENTAGE_THRESHOLD && d2_percentage <= 100) {
      d2 = (d2_percentage * MAX_VEHICLES) / 100;
      setDensityDir2(d2);
      int g2 = compute_green_seconds(d2);
      Serial.print("Parsed density_now_dir2 = "); Serial.print(d2_percentage);
      Serial.print("% -> vehicles = "); Serial.print(d2);
      Serial.print(" -> GREEN2 seconds = "); Serial.println(g2);
    }
  }
  else {
    Serial.println("No valid density fields found in payload.");
    Serial.println("Expected: 'density_now' (0-100) or 'density_now_dir1'/'density_now_dir2' (0-100)");
  }

  Serial.println("====================");
}

void awsLoop() { client.loop(); }
bool awsIsConnected() { return client.connected(); }

void connectToAWS() {
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  client.begin(AWS_IOT_ENDPOINT, 8883, net);
  client.onMessage(messageHandler);

  Serial.print("ESP32 connecting to AWS IoT");
  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();

  client.subscribe(AWS_IOT_TOPIC);
  Serial.println("ESP32 - AWS IoT Connected!");
}

// Note: sendToAWS function removed - this device only receives MQTT messages, does not publish
