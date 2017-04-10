/*
 * poolWritePointCloud.cpp
 *
 *  Created on: 11/05/2013
 *      Author: frivas
 */

#include "poolWritePointCloud.h"
#include <logger/Logger.h>

namespace recorder {

poolWritePointCloud::poolWritePointCloud(Ice::ObjectPrx prx, int freq, size_t poolSize, int pclID,const std::string& baseLogPath):
	RecorderPool(freq,poolSize,pclID),
	PoolPaths(baseLogPath)
{
	this->pointCloudPrx = jderobot::pointCloudPrx::checkedCast(prx);
	if (0== this->pointCloudPrx) {
		LOG(ERROR) << "Invalid proxy";
	}
	createDevicePath(POINTCLOUD, deviceID);
	this->setLogFile(getDeviceLogFilePath(POINTCLOUD, deviceID));
}

poolWritePointCloud::~poolWritePointCloud() {
}


void poolWritePointCloud::consumer_thread(){
	pthread_mutex_lock(&(this->mutex));
	if (this->pointCloud.size()>0){
		std::vector<jderobot::RGBPoint> data2Save(pointCloud[0]);
		this->pointCloud.erase(this->pointCloud.begin());
		long long int relative;
		relative=*(this->its.begin());
		this->its.erase(this->its.begin());
		pthread_mutex_unlock(&(this->mutex));

		std::stringstream relativeString;//create a stringstream
		relativeString << relative;//add number to the stream


		std::string Path= getDeviceLogPath(POINTCLOUD,deviceID) + relativeString.str();
		std::ofstream outfileBinary(Path.c_str(), std::ios::out | std::ios::binary);
		outfileBinary.write((const char *)&data2Save.front(), data2Save.size()*sizeof(jderobot::RGBPoint));
		outfileBinary.close();
		this->logfile << relative << " " << data2Save.size() <<   std::endl;
	}
	else
		pthread_mutex_unlock(&(this->mutex));
	usleep(1000);
}

void poolWritePointCloud::producer_thread(){
	jderobot::pointCloudDataPtr dataPtr=this->pointCloudPrx->getCloudData();


	struct timeval now;
	gettimeofday(&now,NULL);
	long long int relative;
	relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
	pthread_mutex_lock(&(this->mutex));
	while (this->pointCloud.size() > this->poolSize){
		pthread_mutex_unlock(&(this->mutex));
		usleep(100);
		pthread_mutex_lock(&(this->mutex));
	}
	this->pointCloud.push_back(dataPtr->p);
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


	void* poolWritePointCloud::producer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->producer_thread();
			else
				usleep(1000);
		}
		pthread_exit(NULL);
	}

	void* poolWritePointCloud::consumer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->consumer_thread();
			else
				usleep(1000);
		}

		pthread_exit(NULL);
		return NULL;
	}

} /* namespace recorder */
