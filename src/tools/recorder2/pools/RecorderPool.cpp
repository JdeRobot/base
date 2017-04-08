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

    void *RecorderPool::main_pool_consumer_thread(void *foo_ptr) {
        RecorderPoolPtr pool =(*static_cast<RecorderPoolPtr*>(foo_ptr));
        return pool->consumer_thread_imp();
    }

    void *RecorderPool::main_pool_producer_thread(void *foo_ptr) {
        RecorderPoolPtr pool =(*static_cast<RecorderPoolPtr*>(foo_ptr));
        return pool->producer_thread_imp();
    }

    void RecorderPool::setActive(bool value) {
        this->active=value;
    }

    bool RecorderPool::setLogFile(const std::string &logPath) {
        this->logfile.open(logPath.c_str());
    }


}