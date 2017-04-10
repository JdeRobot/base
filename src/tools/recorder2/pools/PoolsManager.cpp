//
// Created by frivas on 4/04/17.
//

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "PoolsManager.h"
#include <logger/Logger.h>


namespace  recorder {

    PoolsManager::PoolsManager(pthread_attr_t &attr, int nConsumers) :
            attr(attr),
            nConsumers(nConsumers)
    {

    }


    PoolsManager::~PoolsManager() {
        for (auto itType = this->poolsByType.begin(), endType = this->poolsByType.end(); itType != endType; ++itType) {
            std::vector<RecorderPoolPtr> &pools = itType->second;
            for (auto itPool = pools.begin(), endPool = pools.end(); itPool != endPool; ++itPool) {
                RecorderPoolPtr& pool = *itPool;
                std::string logFilePath =pool->getLogFilePath();
                LOG(INFO) << "Sorting logFile: " << logFilePath;
                std::stringstream instruction1;
                instruction1 << "sort -n " << logFilePath << " >tempSORT.temp";
                system(instruction1.str().c_str());
                std::stringstream instruction2;
                instruction2 << "mv tempSORT.temp " << logFilePath;
                system(instruction2.str().c_str());
            }
        }
    }




    void PoolsManager::addPool(RECORDER_POOL_TYPE type, RecorderPoolPtr pool) {
        if (this->poolsByType.count(type)) {
            this->poolsByType[type].push_back(pool);
        } else {
            std::vector<RecorderPoolPtr> vector;
            vector.push_back(pool);
            this->poolsByType[type] = vector;
        }

    }

    void PoolsManager::createThreads() {
        for (auto itType = this->poolsByType.begin(), endType = this->poolsByType.end(); itType != endType; ++itType) {
            std::vector<RecorderPoolPtr> &pools = itType->second;
            for (auto itPool = pools.begin(), endPool = pools.end(); itPool != endPool; ++itPool) {
                for (int i = 0; i < nConsumers; i++) {
                    pthread_t thread;
                    pthread_create(&thread, &attr, (*itPool)->main_pool_consumer_thread, &(*itPool));
                    consumerThreads.push_back(thread);
                }
                pthread_t thread;
                pthread_create(&thread, &attr, (*itPool)->main_pool_producer_thread, &(*itPool));
                producerThreads.push_back(thread);
            }
        }

    }

    void PoolsManager::releaseAll() {
        for (auto itType = this->poolsByType.begin(), endType = this->poolsByType.end(); itType != endType; ++itType) {
            std::vector<RecorderPoolPtr> &pools = itType->second;
            for (auto itPool = pools.begin(), endPool = pools.end(); itPool != endPool; ++itPool) {
                (*itPool)->setActive(false);
            }
        }
        for (size_t i = 0; i < consumerThreads.size(); i++) {
            pthread_join(consumerThreads[i], NULL);
        }
        for (size_t i = 0; i < producerThreads.size(); i++) {
            pthread_join(producerThreads[i], NULL);
        }
    }

    std::vector<RecorderPoolPtr> PoolsManager::getPoolsByType(recorder::RECORDER_POOL_TYPE type) {
        if (this->poolsByType.count(type)) {
            return this->poolsByType[type];
        } else {
            return std::vector<recorder::RecorderPoolPtr>();
        }
    }

    void PoolsManager::startRecording(struct timeval& syncTime) {
        for (auto itType = this->poolsByType.begin(), endType = this->poolsByType.end(); itType != endType; ++itType) {
            std::vector<RecorderPoolPtr> &pools = itType->second;
            for (auto itPool = pools.begin(), endPool = pools.end(); itPool != endPool; ++itPool) {
                (*itPool)->setInitialTime(syncTime);
                (*itPool)->setRecording(true);
            }
        }
    }



}
