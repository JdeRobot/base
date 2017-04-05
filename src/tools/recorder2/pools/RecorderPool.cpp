//
// Created by frivas on 3/04/17.
//

#include "RecorderPool.h"


namespace recorder {
    RecorderPool::RecorderPool(int freq, int poolSize, int deviceID) :
            freq(freq),
            poolSize(poolSize),
            deviceID(deviceID)
    {
        this->active=true;
        this->recording=false;
    }

    bool RecorderPool::getActive() {
        return this->active;
    }

    bool RecorderPool::getRecording() {
        return this->recording;
    }

    void RecorderPool::setRecording(bool value) {
        this->recording=value;
    }

    void RecorderPool::setInitialTime(struct timeval &time) {
        this->syncInitialTime=time;
    }


}