#ifndef SOFTWARE_TIME_H
#define SOFTWARE_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
void setTimer(int index, int _counter);
void timerRun(void);
int isTimerExpired(int index);

#ifdef __cplusplus
}
#endif

#endif // SOFTWARE_TIME_H