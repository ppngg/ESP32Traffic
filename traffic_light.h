#ifndef TRAFFIC_LIGHT_H
#define TRAFFIC_LIGHT_H

#ifdef __cplusplus
extern "C" {
#endif

// Optional but useful
void init_traffic_light(void);
void state_machine(void);
// setters called from MQTT
void setDensityDir1(int d);
void setDensityDir2(int d);

// expose your formula so MQTT handler can print computed green time
int compute_green_seconds(int traffic_density);

#ifdef __cplusplus
}
#endif

#endif // TRAFFIC_LIGHT_H
