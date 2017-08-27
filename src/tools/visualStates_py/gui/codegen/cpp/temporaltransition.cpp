#include "temporaltransition.h"
#include <sys/time.h>

TemporalTransition::TemporalTransition(int id, int destinationId, int elapsedTime):Transition(id, destinationId) {
    this->elapsedTime = elapsedTime;
}

void TemporalTransition::init() {
    startTime = getCurrentTime();
}

bool TemporalTransition::checkCondition() {
    long diffTime = getCurrentTime()-startTime;
    if (diffTime > elapsedTime) {
        return true;
    } else {
        return false;
    }
}

long TemporalTransition::getCurrentTime() {
    struct timeval a;
    gettimeofday(&a, 0);
    return (a.tv_sec * 1000000 + a.tv_usec)/1000;
}
