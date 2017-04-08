//
// Created by frivas on 3/04/17.
//

#ifndef JDEROBOT_RECORDERPOOL_H
#define JDEROBOT_RECORDERPOOL_H

#include <time.h>
#include <boost/thread/thread.hpp>
#include <fstream>


namespace recorder {
    class RecorderPool;
    typedef boost::shared_ptr<RecorderPool> RecorderPoolPtr;


    class RecorderPool {
    public:
        RecorderPool(int freq, int poolSize, int deviceID);
        bool getActive();
        bool getRecording();
        void setRecording(bool value);
        void setActive(bool value);
        void setInitialTime(struct timeval& time);
        bool setLogFile(const std::string& logPath);

        static void* main_pool_consumer_thread(void* foo_ptr);

        static void* main_pool_producer_thread(void* foo_ptr);

        virtual void* consumer_thread_imp()=0;

        virtual void* producer_thread_imp()=0;



    protected:
        int freq;
        int poolSize;
        int deviceID;
        float cycle;
        struct timeval lastTime;
        pthread_mutex_t mutex;
        std::vector<long long int> its;
        struct timeval syncInitialTime;
        std::ofstream logfile;



    private:
        bool active;
        bool recording;

    };

}

#endif //JDEROBOT_RECORDERPOOL_H
