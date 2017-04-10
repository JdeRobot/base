/*
 * poolWritePose3d.cpp
 *
 *  Created on: 10/05/2013
 *      Author: frivas
 */

#include "poolWritePose3d.h"
#include <logger/Logger.h>

namespace recorder{


poolWritePose3d::poolWritePose3d(Ice::ObjectPrx prx, int freq, size_t poolSize, int pose3dID, const std::string& baseLogPath):
		RecorderPool(freq,poolSize,pose3dID),
		PoolPaths(baseLogPath)
{
	this->pose3DPrx = jderobot::Pose3DPrx::checkedCast(prx);
	if (0== this->pose3DPrx) {
		LOG(ERROR) << "Invalid proxy";
	}
	createDevicePath(POSE3D, deviceID);
	this->setLogFile(getDeviceLogFilePath(POSE3D, deviceID));
}

poolWritePose3d::~poolWritePose3d() {
}


void poolWritePose3d::consumer_thread(){
	pthread_mutex_lock(&(this->mutex));
	if (this->pose3dVec.size()>0){
		recorder::pose3d data2Save;
		data2Save = this->pose3dVec[0];
		this->pose3dVec.erase(this->pose3dVec.begin());
		long long int relative;
		relative=*(this->its.begin());
		this->its.erase(this->its.begin());
		pthread_mutex_unlock(&(this->mutex));

		std::stringstream relativeString;//create a stringstream
		relativeString << relative;//add number to the stream


		std::string Path= getDeviceLogPath(POSE3D,deviceID) + relativeString.str();
		std::ofstream outfileBinary(Path.c_str(), std::ios::out | std::ios::binary);
		outfileBinary.write((const char *)&data2Save, sizeof(recorder::pose3d));
		outfileBinary.close();

		this->logfile << relative <<   std::endl;

	}
	else
		pthread_mutex_unlock(&(this->mutex));
	usleep(1000);
}

void poolWritePose3d::producer_thread(){
	jderobot::Pose3DDataPtr dataPtr=this->pose3DPrx->getPose3DData();
	recorder::pose3d data;
	data.q0=dataPtr->q0;
	data.q1=dataPtr->q1;
	data.q2=dataPtr->q2;
	data.q3=dataPtr->q3;
	data.x=dataPtr->x;
	data.y=dataPtr->y;
	data.z=dataPtr->z;
	data.h=dataPtr->h;
	struct timeval now;
	gettimeofday(&now,NULL);
	long long int relative;
	relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
	pthread_mutex_lock(&(this->mutex));
	while (this->pose3dVec.size() > this->poolSize){
		pthread_mutex_unlock(&(this->mutex));
		usleep(100);
		pthread_mutex_lock(&(this->mutex));
	}
	this->pose3dVec.push_back(data);
	this->its.push_back(relative);
	pthread_mutex_unlock(&(this->mutex));
	gettimeofday(&now,NULL);

	long long int totalNow=now.tv_sec*1000000+now.tv_usec;
	long long int totalLast=lastTime.tv_sec*1000000+lastTime.tv_usec;

	double sleepTime =this->cycle - (totalNow-totalLast)/1000.;

	if(sleepTime < 0 )
		sleepTime = 0;
	usleep(sleepTime*1000);
	gettimeofday(&lastTime,NULL);
}


	void* poolWritePose3d::producer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->producer_thread();
			else
				usleep(1000);
		}
		pthread_exit(NULL);
	}

	void* poolWritePose3d::consumer_thread_imp(){
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
