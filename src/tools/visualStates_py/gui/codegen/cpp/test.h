#ifndef TEST_H
#define TEST_H

#include "state.h"
#include "temporaltransition.h"
#include "conditionaltransition.h"
#include "interfaces.h"

#include <jderobot/motors.h>

class State0 : public State {
public:
    State0(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class State1 : public State {
public:
    State1(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class State2 : public State {
public:
    State2(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):State(id, initial, interfaces, cycleDuration, parent, gui) {}
    virtual void runCode();
};

class Tran0 : public ConditionalTransition {
public:
    Tran0(int id, int destId, Interfaces* interfaces):ConditionalTransition(id, destId, interfaces) {}
    virtual void init();
    virtual bool checkCondition();
    virtual void runCode();
};

class MyInterfaces : public Interfaces {
public:
    jderobot::MotorsPrx myMotors;

    virtual void connectProxies(int argc, char* argv[]);
    virtual void destroyProxies();

};

#endif