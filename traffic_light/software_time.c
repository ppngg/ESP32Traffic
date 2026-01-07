#include "software_time.h"

// Actual definitions (will be in only one object file)
static int timerCounter[10];
static int timerFlag[10];

void setTimer(int index, int _counter) {
    timerCounter[index] = _counter;
    timerFlag[index] = 0;
}

void timerRun(void) {
    for (int i = 0; i < 10; i++) {
        if (timerCounter[i] > 0) {
            timerCounter[i]--;
            if (timerCounter[i] <= 0) {
                timerFlag[i] = 1;
            }
        }
    }
}

int isTimerExpired(int index) {
    if (timerFlag[index] == 1) {
        timerFlag[index] = 0;
        return 1;
    }

    return 0;
} 