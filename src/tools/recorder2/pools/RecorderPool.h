//
// Created by frivas on 3/04/17.
//

#ifndef JDEROBOT_RECORDERPOOL_H
#define JDEROBOT_RECORDERPOOL_H

#include <time.h>
#include <boost/thread/thread.hpp>


namespace recorder {
    class RecorderPool {
    public:
        RecorderPool(int freq, int poolSize, int deviceID);
        bool getActive();
        bool getRecording();
        void setRecording(bool value);
        void setInitialTime(struct timeval& time);

    protected:
        int freq;
        int poolSize;
        int deviceID;
        float cycle;
        struct timeval lastTime;
        pthread_mutex_t mutex;
        std::vector<long long int> its;
        struct timeval syncInitialTime;


    private:
        bool active;
        bool recording;

    };


}

#endif //JDEROBOT_RECORDERPOOL_H
