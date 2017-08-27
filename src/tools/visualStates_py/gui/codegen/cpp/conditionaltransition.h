#ifndef CONDITONALTRANSITION_H
#define CONDITONALTRANSITION_H

#include "transition.h"

class ConditionalTransition : public Transition {
public:
    ConditionalTransition(int id, int destId):Transition(id, destId){}
};

#endif
