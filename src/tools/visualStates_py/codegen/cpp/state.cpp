#include "state.h"
#include <unistd.h>
#include <sys/time.h>
#include <iostream>

State::State(int id, bool initial, int cycleDuration, State* parent, RunTimeGui* gui) {
    this->id = id;
    this->initial = initial;
    this->cycleDuration = cycleDuration;
    this->parent = parent;
    this->gui = gui;
    running = true;

    if (parent != NULL) {
        parent->addState(this);
    }
}

void State::init() {
    for (unsigned int i = 0; i < transitions.size(); i++) {
        transitions.at(i)->init();
    }

    if (gui != NULL) {
        gui->emitRunningStateById(id);
    }
}

void State::addState(State* state) {
    if (state->initial) {
        currentState = state;
    }

    states.push_back(state);
    statesById.insert(std::pair<int, State*>(state->id, state));
}

void State::addTransition(Transition* tran) {
    transitions.push_back(tran);
}

void* State::threadRunner(void* owner) {
    ((State*)owner)->run();
}

void State::startThread() {
    pthread_create(&thread, NULL, &State::threadRunner, this);
}

void State::run() {
    bool initState = true;
    while (running) {
        long startTime = getCurrentTime();

        bool runState = false;
        if (parent != NULL) {
            if (parent->currentState == this) {
                runState = true;
            }
        } else if (parent == NULL) {
            runState = true;
        }

        if (initState) {
            currentState->init();
            initState = false;
        }

        if (runState) {
            // transition evaluations
            for (unsigned int i = 0; i < currentState->transitions.size(); i++) {
                Transition* tran = currentState->transitions.at(i);
                if (tran->checkCondition()) {
                    tran->runCode();
                    currentState = statesById[tran->getDestinationId()];
                    currentState->init();
                }
            }
            // user code of the state
            currentState->runCode();
        }

        long finishTime = getCurrentTime();
        long elapsedTime = (finishTime - startTime) / 1000;
        if (elapsedTime < cycleDuration) {
            elapsedTime = cycleDuration - elapsedTime;
            // convert milliseconds to microseconds
            usleep(elapsedTime*1000);
        }
    }
}

void State::stop() {
    running = false;
}

long State::getCurrentTime() {
    struct timeval a;
    gettimeofday(&a, NULL);
    return (a.tv_sec * 1000000 + a.tv_usec)/1000;
}

void State::join() {
    pthread_join(thread, NULL);
}
