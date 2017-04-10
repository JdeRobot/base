/*
 * poolWriteEncoders.cpp
 *
 *  Created on: 23/07/2013
 *      Author: frivas
 */
#include <logger/Logger.h>

#include "poolWriteEncoders.h"

namespace recorder {

poolWriteEncoders::poolWriteEncoders(Ice::ObjectPrx prx, int freq, int poolSize, int encoderID,const std::string& baseLogPath):
		RecorderPool(freq,poolSize,encoderID),
		PoolPaths(baseLogPath)
{
	this->encodersPrx = jderobot::EncodersPrx::checkedCast(prx);
	if (0== this->encodersPrx) {
		LOG(ERROR) << "Invalid proxy";
	}
	createDevicePath(ENCODERS, deviceID);
	this->setLogFile(getDeviceLogFilePath(ENCODERS, deviceID));
}

poolWriteEncoders::~poolWriteEncoders() {

}


void poolWriteEncoders::consumer_thread(){
	pthread_mutex_lock(&(this->mutex));
	if (this->encoders.size()>0){
		recorder::encoders data2Save;
		data2Save = this->encoders[0];
		this->encoders.erase(this->encoders.begin());
		long long int relative;
		relative=*(this->its.begin());
		this->its.erase(this->its.begin());
		pthread_mutex_unlock(&(this->mutex));

		std::stringstream relativeString;//create a stringstream
		relativeString << relative;//add number to the stream


		std::string Path= getDeviceLogPath(ENCODERS,deviceID) + relativeString.str();
		std::ofstream outfileBinary(Path.c_str(), std::ios::out | std::ios::binary);
		outfileBinary.write((const char *)&data2Save, sizeof(recorder::encoders));
		outfileBinary.close();

		this->logfile << relative <<   std::endl;

	}
	else
		pthread_mutex_unlock(&(this->mutex));
	usleep(1000);
}

void poolWriteEncoders::producer_thread(){
	jderobot::EncodersDataPtr dataPtr=this->encodersPrx->getEncodersData();
	recorder::encoders data;

	data.robotx=dataPtr->robotx;
	data.roboty=dataPtr->roboty;
	data.robottheta=dataPtr->robottheta;
	data.robotcos=dataPtr->robotcos;
	data.robotsin=dataPtr->robotsin;

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

	if(sleepTime < 0 )
		sleepTime = 0;
	usleep(sleepTime*1000);
	gettimeofday(&lastTime,NULL);
}


	void* poolWriteEncoders::producer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->producer_thread();
			else
				usleep(1000);
		}
		pthread_exit(NULL);
	}

	void* poolWriteEncoders::consumer_thread_imp(){
		while (this->getActive()){
			if (this->getRecording())
				this->consumer_thread();
			else
				usleep(1000);
		}

		pthread_exit(NULL);
		return NULL;
	}



} /* namespace gbRGBD2PC */
