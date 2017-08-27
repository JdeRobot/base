#include "transition.h"

Transition::Transition(int id, int destinationId) {
    this->id = id;
    this->destinationId = destinationId;
}

int Transition::getDestinationId() {
    return destinationId;
}