#ifndef TEMPORALTRANSITION_H
#define TEMPORALTRANSITION_H

#include "transition.h"

class TemporalTransition : public Transition {
protected:
    int elapsedTime;
    long startTime;

    long getCurrentTime();

public:
    TemporalTransition(int id, int destinationId, int elapsedTime);
    virtual void init();
    virtual bool checkCondition();

    virtual void runCode() = 0;
};


#endif
