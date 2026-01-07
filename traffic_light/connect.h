#ifndef CONNECT_H
#define CONNECT_H

#ifdef __cplusplus
extern "C" {
#endif

// AWS connect + MQTT loop
void connectToAWS();
void awsLoop();
bool awsIsConnected();

// Note: This device only receives MQTT messages, does not publish

#ifdef __cplusplus
}
#endif

#endif // TRAFFIC_LIGHT_H
