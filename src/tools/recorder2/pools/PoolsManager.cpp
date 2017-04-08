//
// Created by frivas on 4/04/17.
//

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "PoolsManager.h"


namespace  recorder {

    PoolsManager::PoolsManager(pthread_attr_t &attr, int nConsumers,const std::string& baseLogPath) :
            attr(attr),
            nConsumers(nConsumers),
            baseLogPath(baseLogPath)
    {
        this->typesSufix[LASERS]="lasers";
        this->typesSufix[POSE3DENCODERS]="pose3dencoders";
        this->typesSufix[POSE3D]="pose3d";
        this->typesSufix[ENCODERS]="encoders";
        this->typesSufix[POINTCLOUD]="pointClouds";
        this->typesSufix[IMAGES]="images";

        this->typesSufixIndependent[LASERS]="laser";
        this->typesSufixIndependent[POSE3DENCODERS]="pose3dencoder";
        this->typesSufixIndependent[POSE3D]="pose3d";
        this->typesSufixIndependent[ENCODERS]="encoder";
        this->typesSufixIndependent[POINTCLOUD]="pointCloud";
        this->typesSufixIndependent[IMAGES]="camera";


        testPathAndCreateIfNotExists(baseLogPath);

    }


    void PoolsManager::addPool(RECORDER_POOL_TYPE type, RecorderPoolPtr pool) {

        std::string separator=getPathSeparator();
        std::string typeMainTypePath = this->baseLogPath + separator + this->typesSufix[type];
        testPathAndCreateIfNotExists(typeMainTypePath);


        int index=1;
        if (this->poolsByType.count(type)) {
            index=this->poolsByType[type].size() + 1;
        }
        std::stringstream ss;
        ss << index;
        testPathAndCreateIfNotExists(typeMainTypePath + separator + this->typesSufixIndependent[type] + ss.str());

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


        for (int i = 0; i < consumerThreads.size(); i++) {
            pthread_join(consumerThreads[i], NULL);
        }
        for (int i = 0; i < producerThreads.size(); i++) {
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
        std::string separator=getPathSeparator();
        for (auto itType = this->poolsByType.begin(), endType = this->poolsByType.end(); itType != endType; ++itType) {
            std::vector<RecorderPoolPtr> &pools = itType->second;
            for (auto itPool = pools.begin(), endPool = pools.end(); itPool != endPool; ++itPool) {
                int index = (int)std::distance(pools.begin(),itPool) + 1;
                std::stringstream ss;
                ss << this->baseLogPath << separator << this->typesSufix[itType->first] << separator << this->typesSufixIndependent[itType->first] << index << separator << this->typesSufixIndependent[itType->first] << "Data.jde";
                (*itPool)->setLogFile(ss.str());
                (*itPool)->setInitialTime(syncTime);
                (*itPool)->setRecording(true);
            }
        }
    }

    std::string PoolsManager::getPathSeparator() {
        return "/";
    }

    void PoolsManager::testPathAndCreateIfNotExists(const std::string &path) {
        boost::filesystem::path p(path);
        if (!boost::filesystem::exists(p)){
            boost::filesystem::create_directories(p);
        }
    }


}