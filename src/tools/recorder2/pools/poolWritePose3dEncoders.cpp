/*
 * poolWritePose3dEncoders.cpp
 *
 *  Created on: 10/05/2013
 *      Author: frivas
 */

#include "poolWritePose3dEncoders.h"
#include <logger/Logger.h>

namespace recorder{


poolWritePose3dEncoders::poolWritePose3dEncoders(Ice::ObjectPrx prx, int freq, int poolSize, int encoderID,const std::string& baseLogPath):
		RecorderPool(freq,poolSize,encoderID),
		PoolPaths(baseLogPath)
{
	this->pose3DEncodersPrx = jderobot::Pose3DEncodersPrx::checkedCast(prx);
	if (0== this->pose3DEncodersPrx) {
		LOG(ERROR) << "Invalid proxy";
	}
	createDevicePath(POSE3DENCODERS, deviceID);
	this->setLogFile(getDeviceLogFilePath(POSE3DENCODERS, deviceID));
}

poolWritePose3dEncoders::~poolWritePose3dEncoders() {
}


void poolWritePose3dEncoders::consumer_thread(){
//	while(this->active){


		pthread_mutex_lock(&(this->mutex));
		if (this->encoders.size()>0){
			//std::cout << " camara: " << cameraID <<  this->images.size()  << std::endl;

			recorder::pose3dencoders data2Save;
			data2Save = this->encoders[0];
			this->encoders.erase(this->encoders.begin());
			long long int relative;
			relative=*(this->its.begin());
			this->its.erase(this->its.begin());
			pthread_mutex_unlock(&(this->mutex));


			std::stringstream idString;//create a stringstream
			idString << this->deviceID;//add number to the stream
			std::stringstream relativeString;//create a stringstream
			relativeString << relative;//add number to the stream

			std::string Path= getDeviceLogPath(POSE3DENCODERS,deviceID) + relativeString.str();
			std::ofstream outfileBinary(Path.c_str(), std::ios::out | std::ios::binary);
			outfileBinary.write((const char *)&data2Save, sizeof(recorder::pose3dencoders));
			outfileBinary.close();

			this->logfile << relative <<   std::endl;

		}
		else
			pthread_mutex_unlock(&(this->mutex));
		usleep(1000);


//	}
}

void poolWritePose3dEncoders::producer_thread(){
	//std::cout << "productor entro" << std::endl;
//	while(this->active){
		jderobot::Pose3DEncodersDataPtr dataPtr=this->pose3DEncodersPrx->getPose3DEncodersData();
		recorder::pose3dencoders data;
		data.clock=dataPtr->clock;
		data.maxPan=dataPtr->maxPan;
		data.maxTilt=dataPtr->maxTilt;
		data.minPan=dataPtr->minPan;
		data.minTilt=dataPtr->minTilt;
		data.pan=dataPtr->pan;
		data.roll=dataPtr->roll;
		data.tilt=dataPtr->tilt;
		data.x=dataPtr->x;
		data.y=dataPtr->y;
		data.z=dataPtr->z;

		struct timeval now;
		gettimeofday(&now,NULL);
		long long int relative;
		relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
		pthread_mutex_lock(&(this->mutex));
		while (this->encoders.size() > this->poolSize){
			pthread_mutex_unlock(&(this->mutex));
			usleep(100);
			pthread_mutex_lock(&(this->mutex));
		}
		this->encoders.push_back(data);
		this->its.push_back(relative);
		pthread_mutex_unlock(&(this->mutex));
		gettimeofday(&now,NULL);

		long long int totalNow=now.tv_sec*1000000+now.tv_usec;
		long long int totalLast=lastTime.tv_sec*1000000+lastTime.tv_usec;

		float sleepTime =this->cycle - (totalNow-totalLast)/1000.;

		//std::cout << "productor: " << this->cameraID << ", sleep: " << sleepTime << std::endl;
		if(sleepTime < 0 )
			sleepTime = 0;
		usleep(sleepTime*1000);
		gettimeofday(&lastTime,NULL);
		//std::cout << "productor salgo" << std::endl;
//	}
}


	void* poolWritePose3dEncoders::producer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->producer_thread();
			else
				usleep(1000);
		}
		pthread_exit(NULL);
	}

	void* poolWritePose3dEncoders::consumer_thread_imp(){
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
