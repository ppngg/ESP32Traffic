#include "traffic_light.h"
#include "software_time.h"
#include <Arduino.h>

#define D3 6
#define D4 7
#define D5 8
#define D6 9

#define STATE1 1
#define STATE2 2
#define STATE3 3
#define STATE4 4

int state = STATE1;

// ---------- Dynamic densities (update these from MQTT or sensors) ----------
static volatile int density_dir1 = 50; // used for GREEN1 (STATE3)
static volatile int density_dir2 = 50; // used for GREEN2 (STATE1)

void setDensityDir1(int d) { if (d < 0) d = 0; density_dir1 = d; }
void setDensityDir2(int d) { if (d < 0) d = 0; density_dir2 = d; }

// ---------- Your formula: returns green time (SECONDS) ----------
int compute_green_seconds(int traffic_density) {
  int avg_speed;
  int sat_flow;
  int cycle_base;

  if (traffic_density <= 50) {
    avg_speed  = 40;
    sat_flow   = 1800;
    cycle_base = 90;
  } else if (traffic_density <= 100) {
    avg_speed  = 20;
    sat_flow   = 1500;
    cycle_base = 120;
  } else {
    return 40; // fallback seconds
  }

  long demand_flow = (long)traffic_density * (long)avg_speed;

  // precedence fixed
  long green = (demand_flow * (long)cycle_base) / (2L * (long)sat_flow);

  if (green < 1) green = 1;
  return (int)green; // seconds
}

// Convert seconds to timer ticks.
// timerRun() is called every 10ms (scheduler tick), so:
// 1 second = 100 ticks (1000ms / 10ms = 100)
static int green_to_timer_units(int green_sec) {
  // Convert seconds to ticks: 1 second = 100 ticks (at 10ms per tick)
  long ticks = (long)green_sec * 100L;
  if (ticks < 1) ticks = 1;
  if (ticks > 60000L) ticks = 60000L; // clamp to 10 min for safety (6000 seconds * 100)
  return (int)ticks;
}

// ---------- Output states (unchanged) ----------

// RED1 GREEN2
void state1() {
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, LOW);
}

// RED1 yellow2
void state2() {
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH);
  digitalWrite(D5, LOW);
  digitalWrite(D6, HIGH);
}

// GREEN1 RED2
void state3() {
  digitalWrite(D3, HIGH);
  digitalWrite(D4, LOW);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, HIGH);
}

// yellow1 red2
void state4() {
  digitalWrite(D3, LOW);
  digitalWrite(D4, HIGH);
  digitalWrite(D5, HIGH);
  digitalWrite(D6, HIGH);
}

void state_machine(void) {
  const int YELLOW_TIME = 200; // keep your static yellow timing (same as before)

  switch (state) {
    case STATE1:
      // GREEN2 phase -> dynamic based on density_dir2
      if (isTimerExpired(0)) {
        state = STATE2;
        state2();
        setTimer(0, YELLOW_TIME);
      }
      break;

    case STATE2:
      // switch to GREEN1 phase -> set dynamic timer for STATE3 green
      if (isTimerExpired(0)) {
        state = STATE3;
        state3();

        int gsec = compute_green_seconds(density_dir1);
        setTimer(0, green_to_timer_units(gsec));
      }
      break;

    case STATE3:
      // GREEN1 phase ends -> yellow1
      if (isTimerExpired(0)) {
        state = STATE4;
        state4();
        setTimer(0, YELLOW_TIME);
      }
      break;

    case STATE4:
      // switch back to GREEN2 phase -> set dynamic timer for STATE1 green
      if (isTimerExpired(0)) {
        state = STATE1;
        state1();

        int gsec = compute_green_seconds(density_dir2);
        setTimer(0, green_to_timer_units(gsec));
      }
      break;

    default:
      state = STATE1;
      state1();
      setTimer(0, 300);
      break;
  }
}

void init_traffic_light(void) {
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);

  // Start in STATE1 (GREEN2) with dynamic time
  state = STATE1;
  state1();

  int gsec = compute_green_seconds(density_dir2);
  setTimer(0, green_to_timer_units(gsec));
}
