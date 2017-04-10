/*
 * poolWriteLasers.cpp
 *
 *  Created on: 10/05/2013
 *      Author: frivas
 */

#include "poolWriteLasers.h"
#include <logger/Logger.h>


namespace recorder{
poolWriteLasers::poolWriteLasers(Ice::ObjectPrx prx, int freq, size_t poolSize, int laserID, const std::string& baseLogPath):
		RecorderPool(freq,poolSize,laserID),
		PoolPaths(baseLogPath)
{
	this->laserPrx = jderobot::LaserPrx::checkedCast(prx);
	if (0== this->laserPrx) {
		LOG(ERROR) << "Invalid proxy";
	}
    createDevicePath(LASERS, deviceID);
    this->setLogFile(getDeviceLogFilePath(LASERS, deviceID));
}

poolWriteLasers::~poolWriteLasers() {
}

void poolWriteLasers::consumer_thread(){
    pthread_mutex_lock(&(this->mutex));
    if (this->lasers.size()>0){
        std::vector<int> data2Save(lasers[0].values);
        int maxRange = lasers[0].maxRange;
        int minRange = lasers[0].minRange;
        float minAngle = lasers[0].minAngle;
        float maxAngle = lasers[0].maxAngle;
        this->lasers.erase(this->lasers.begin());
        long long int relative;
        relative=*(this->its.begin());
        this->its.erase(this->its.begin());
        pthread_mutex_unlock(&(this->mutex));


        std::stringstream relativeString;//create a stringstream
        relativeString << relative;//add number to the stream


        std::string Path= getDeviceLogPath(LASERS,deviceID) + relativeString.str();

        std::ofstream outfileBinary(Path.c_str(), std::ios::out | std::ios::binary);
        outfileBinary.write((const char *)&data2Save.front(), data2Save.size()*sizeof(int));
        outfileBinary.close();

        this->logfile << relative << " " << data2Save.size() << " " << maxRange << " " << minRange << " " << maxAngle << " " << minAngle <<   std::endl;
    }
    else
        pthread_mutex_unlock(&(this->mutex));
    usleep(1000);
}

void poolWriteLasers::producer_thread(){
    jderobot::LaserDataPtr dataPtr=this->laserPrx->getLaserData();
    LaserD data;

    data.values = dataPtr->distanceData;
    data.minAngle = dataPtr->minAngle;
    data.maxAngle = dataPtr->maxAngle;
    data.minRange = dataPtr->minRange;
    data.maxRange = dataPtr->maxRange;

    struct timeval now;
    gettimeofday(&now,NULL);
    long long int relative;
    relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
    pthread_mutex_lock(&(this->mutex));
    while (this->lasers.size() > (size_t)this->poolSize){
        pthread_mutex_unlock(&(this->mutex));
        usleep(100);
        pthread_mutex_lock(&(this->mutex));
    }
    this->lasers.push_back(data);
    this->its.push_back(relative);
    pthread_mutex_unlock(&(this->mutex));
    gettimeofday(&now,NULL);

    long long int totalNow=now.tv_sec*1000000+now.tv_usec;
    long long int totalLast=lastTime.tv_sec*1000000+lastTime.tv_usec;

    float sleepTime =this->cycle - (totalNow-totalLast)/1000.;

    if(sleepTime < 0 )
        sleepTime = 0;
    usleep(sleepTime*1000);
    gettimeofday(&lastTime,NULL);
}


    void* poolWriteLasers::producer_thread_imp(){
        while (this->getActive()){
            if (this->getRecording())
                this->producer_thread();
            else
                usleep(1000);
        }
        pthread_exit(NULL);
    }

    void* poolWriteLasers::consumer_thread_imp(){
        while (this->getActive()){
            if (this->getRecording())
                this->consumer_thread();
            else
                usleep(1000);
        }

        pthread_exit(NULL);
        return NULL;
    }


} //namespace


