#include <Arduino.h>
#include <WiFi.h>
#include "secrects.h"
#include "connect.h"
#include "scheduler.h"
#include "traffic_light.h"
#include "software_time.h"

// Task function wrappers for scheduler
void awsLoopTask(void) {
  awsLoop();
}

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("ESP32 connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected. IP: ");
  Serial.println(WiFi.localIP());

  connectToAWS();

  // Initialize scheduler
  SCH_Init();

  // Initialize traffic light hardware
  init_traffic_light();

  // Add tasks to scheduler:
  // timerRun: period 1 (runs every tick to decrement timers)
  SCH_Add_Task(timerRun, 0, 1);

  // state_machine: period 10 (runs every 10 ticks to check state transitions)
  SCH_Add_Task(state_machine, 0, 10);

  // awsLoopTask: period 50 (runs every 50 ticks for MQTT loop)
  SCH_Add_Task(awsLoopTask, 0, 50);

  Serial.println("Scheduler initialized with traffic light state machine");
  Serial.println("Subscribed to topic: esp32/traffic/density_now");
  Serial.println("Waiting for MQTT messages with density_now (0-100) or density_now_dir1/dir2");
}

void loop() {
  // Update scheduler (decrements delays)
  SCH_Update();
  
  // Dispatch ready tasks
  SCH_Dispatch_Tasks();
  
  // 10ms delay = 1 scheduler tick (INTERRUPT_CYCLE is 10ms)
  delay(10);
}
