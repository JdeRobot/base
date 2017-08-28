#ifndef TRANSITION_H
#define TRANSITION_H

class Transition {
protected:
    int id;
    int destinationId;

public:
    Transition(int id, int destinationId);

    int getDestinationId();

    virtual void init() = 0;
    virtual void runCode() = 0;
    virtual bool checkCondition() = 0;
};

#endif