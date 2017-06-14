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

    enum MODE
    {
        WRITE_FRAME = 0,
        SAVE_BUFFER,
        WRITE_BUFFER,
        WRITE_END_LOG
    };


    class RecorderPool {
    public:
        RecorderPool(int freq, size_t poolSize, int deviceID);
        bool getActive();
        bool getRecording();
        void setRecording(bool value);
        void setActive(bool value);
        void setInitialTime(struct timeval& time);
        bool setLogFile(const std::string& logPath);
        std::string getLogFilePath();

        static void* main_pool_consumer_thread(void* foo_ptr);

        static void* main_pool_producer_thread(void* foo_ptr);

        virtual void* consumer_thread_imp()=0;

        virtual void* producer_thread_imp()=0;



    protected:
        int freq;
        size_t poolSize;
        int deviceID;
        double cycle;
        struct timeval lastTime;
        pthread_mutex_t mutex;
        std::vector<long long int> its;
        struct timeval syncInitialTime;
        std::ofstream logfile;
        std::string logFilePath;



    private:
        bool active;
        bool recording;

    };

}

#endif //JDEROBOT_RECORDERPOOL_H
