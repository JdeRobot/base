/*
 * control.cpp
 *
 *  Created on: 16/05/2013
 *      Author: frivas
 */

#include "control.h"

namespace replayer {

control::control(long long int initState, bool play_in, bool repeat_in) {
	this->nProcFinished=0;
	this->play=play_in;
	this->paused=false;
	this->repeat=repeat_in;
	this->newTime=initState;
	this->nRepetitions=0;
	if (play_in){
		this->status=jderobot::PLAYING;
	}
	else{
		this->status=jderobot::WAITING;
	}

}

control::~control() {
	// TODO Auto-generated destructor stub
}

void control::lock(){
	this->controlMutex.lock();
}

void control::unlock(){
	this->controlMutex.unlock();
}

bool control::getPlay(){
	bool localPlay;
	this->controlMutex.lock();
	localPlay=this->play;
	this->controlMutex.unlock();
	return localPlay;
}

long long int control::getSyncTime(){
	long long int localTime;
	this->controlMutex.lock();
	localTime=this->newTime;
	this->controlMutex.unlock();
	return localTime;
}

long long int control::getRelativeTime(){
	long long int localTime;
	this->controlMutex.lock();
	if (this->play)
		localTime=IceUtil::Time::now().toMilliSeconds() - this->newTime;
	else
		localTime=this->timeToResume;
	this->controlMutex.unlock();
	return localTime;
}

long long int control::wait(){
	long long int localTime;
	{
		IceUtil::Mutex::Lock sync(this->controlMutex);
		this->nProcFinished=this->nProcFinished+1;
		this->sem.wait(sync);
		localTime=this->newTime;
		this->nRepetitions=this->nRepetitions+1;
	}
	return localTime;
}

void control::checkStatus(){
	this->controlMutex.lock();
	if (this->nProcFinished != this->nProcess){
	}
	else{
		if (this->repeat){
			LOG(INFO) << "-------------- STARTING THE LOG AGAIN --------------";
			IceUtil::Time now = IceUtil::Time::now();
			long long int nowInt=(now.toMicroSeconds())/1000;
			this->newTime=nowInt;
			this->nProcFinished=0;
			this->sem.broadcast();
		}
		this->status=jderobot::FINISHED;
        LOG(INFO) << "-------------- FINISHED --------------";

	}
	this->controlMutex.unlock();
}

void control::stop(){
	this->controlMutex.lock();
		this->play=false;
		this->paused=true;
		IceUtil::Time now = IceUtil::Time::now();
		long long int nowInt=(now.toMicroSeconds())/1000;
		//std::cout << "now_ " << nowInt << std::endl;

		this->timeToResume=nowInt-this->newTime;
	this->controlMutex.unlock();
}

void control::resume(){
	this->controlMutex.lock();
		this->play=true;
		this->paused=false;

		IceUtil::Time now = IceUtil::Time::now();
		long long int nowInt=(now.toMicroSeconds())/1000;
		//std::cout << "now_ " << nowInt << std::endl;
		//std::cout << "TIEMPO RELATIVO: " << this->timeToResume << std::endl;
		this->newTime=nowInt-this->timeToResume;
	this->controlMutex.unlock();
}

void control::setRepeat(bool active){
	this->repeat=active;
}

void control::setProcesses(int procs){
	this->nProcess=procs;
}

void control::setStep(int step){
	this->controlMutex.lock();
		IceUtil::Time n = IceUtil::Time::now();
		long long int nowInt=(n.toMicroSeconds())/1000;
		this->timeToResume=this->timeToResume  + step;
		this->newTime=nowInt-this->timeToResume;
	this->controlMutex.unlock();
}


jderobot::ReplayerStatus control::getstatus(){
	if (status==jderobot::FINISHED){
	}
	else if (this->paused){
		status=jderobot::PAUSED;
	}
	else if (!play){
		status=jderobot::WAITING;
	}
	else{
		status=jderobot::PLAYING;
	}
	return status;

}


} /* namespace replayer */

