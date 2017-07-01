//
// Created by frivas on 4/04/17.
//

#ifndef JDEROBOT_POOLSMANAGER_H
#define JDEROBOT_POOLSMANAGER_H

#include <recorder/Common.h>
#include "RecorderPool.h"

namespace recorder {
    class PoolsManager {
    public:
        PoolsManager(pthread_attr_t& attr, int nConsumers);
        ~PoolsManager();
        void addPool(RECORDER_POOL_TYPE type, RecorderPoolPtr pool);
        void createThreads();
        void releaseAll();
        std::vector<RecorderPoolPtr> getPoolsByType(RECORDER_POOL_TYPE type);
        void startRecording(struct timeval& syncTime);


    private:
        std::map<RECORDER_POOL_TYPE, std::vector<RecorderPoolPtr>> poolsByType;



        pthread_attr_t& attr;
        int nConsumers;
        std::vector<pthread_t> consumerThreads;
        std::vector<pthread_t> producerThreads;

    };

    typedef boost::shared_ptr<PoolsManager> PoolsManagerPtr;
}


#endif //JDEROBOT_POOLSMANAGER_H
