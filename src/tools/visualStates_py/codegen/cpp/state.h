#ifndef STATE_H
#define STATE_H

#include <vector>
#include <map>
#include "transition.h"
#include "runtimegui.h"

class State {
protected:
    int id;
    bool active;
    bool running;
    State* parent;
    State* currentState;
    bool initial;
    bool displayGui;
    int cycleDuration;

    std::vector<State*> states;
    std::vector<Transition*> transitions;
    std::map<int, State*> statesById;

    RunTimeGui* gui;
    pthread_t thread;

public:
    State(int id, bool initial, int cycleDuration, State* parent, RunTimeGui* gui);

    void init();
    void addState(State* state);
    void addTransition(Transition* transition);
    void startThread();
    void run();
    void stop();
    void join();
    long getCurrentTime();

    virtual void runCode() {}

    static void* threadRunner(void*);




};

#endif