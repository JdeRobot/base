/*
 *  Copyright (C) 1997-2013 JDE Developers TeamkinectViewer.camRGB
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Author : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */
#include "jderobot/comm/ice/navdataIceClient.hpp"

namespace Comm {

NavdataIceClient::NavdataIceClient(Comm::Communicator* jdrc, std::string prefix) {

	this->prefix=prefix;
	
	this->refreshRate=0;
	float fps=jdrc->getConfig().asFloatWithDefault(prefix+".Fps",10);
	this->cycle=(1/fps)*1000000;


	std::string proxy = jdrc->getConfig().asString(prefix+".Proxy");
	Ice::ObjectPrx baseNavdata = jdrc->getIceComm()->stringToProxy(proxy);

	if (0==baseNavdata){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::NavdataPrx::checkedCast(baseNavdata);

			if (0 == this->prx){
				this->on = false;
	 	   		std::cout <<"Invalid proxy "+ prefix + ".Proxy" <<std::endl;
	 		}else{
	 			this->on = true;
	 			std::cout << prefix + " connected" << std::endl;
	 		}

		
		}catch (const Ice::ConnectionRefusedException& e) {
			std::cout << prefix +" inactive" << std::endl;
		}
		catch (const Ice::Exception& ex) {
			std::cerr << ex << std::endl;
		}
	}

	this->pauseStatus=false;

}

NavdataIceClient::~NavdataIceClient() {
	this->on=false;
}


void NavdataIceClient::pause(){
	std::cout << "Paused" << std::endl;
	this->pauseStatus=true;
}

void NavdataIceClient::resume(){
	std::cout << "resumed" << std::endl;
	this->controlMutex.lock();
		this->pauseStatus=false;
		this->sem.broadcast();
	this->controlMutex.unlock();
}

void NavdataIceClient::run(){

	IceUtil::Time last;

	last=IceUtil::Time::now();
	while (this->on){
		if (pauseStatus){
			IceUtil::Mutex::Lock sync(this->controlMutex);
			this->sem.wait(sync);
		}

		try{
			jderobot::NavdataDataPtr localNavdata=this->prx->getNavdata();


			this->controlMutex.lock();
			this->navdataData.vehicle = localNavdata->vehicle;
    		this->navdataData.state = localNavdata->state;
    		
    		this->navdataData.batteryPercent = localNavdata->batteryPercent;
    		this->navdataData.magX = localNavdata->magX;
    		this->navdataData.magY = localNavdata->magY;
    		this->navdataData.magZ = localNavdata->magZ;
    		this->navdataData.pressure = localNavdata->pressure;
    		this->navdataData.temp = localNavdata->temp;
    		
    		this->navdataData.windSpeed = localNavdata->windSpeed;
    		this->navdataData.windAngle = localNavdata->windAngle;
    		this->navdataData.windCompAngle = localNavdata->windCompAngle;
    		
    		this->navdataData.rotX = localNavdata->rotX;
    		this->navdataData.rotY = localNavdata->rotY;
    		this->navdataData.rotZ = localNavdata->rotZ;
    		this->navdataData.altd = localNavdata->altd;

    		this->navdataData.vx = localNavdata->vx;
    		this->navdataData.vy = localNavdata->vy;
    		this->navdataData.vz = localNavdata->vz;
    		this->navdataData.ax = localNavdata->ax;
    		this->navdataData.ay = localNavdata->ay;
    		this->navdataData.az = localNavdata->az;

    		this->navdataData.tagsCount = localNavdata->tagsCount;
    		this->navdataData.tagsType = localNavdata->tagsType;
    		this->navdataData.tagsXc = localNavdata->tagsXc;
    		this->navdataData.tagsYc = localNavdata->tagsYc;
    		this->navdataData.tagsWidth = localNavdata->tagsWidth;
    		this->navdataData.tagsHeight = localNavdata->tagsHeight;
    		this->navdataData.tagsOrientation = localNavdata->tagsOrientation;
    		this->navdataData.tagsDistance = localNavdata->tagsDistance;

    		this->navdataData.timeStamp = localNavdata->tm;
			

			this->controlMutex.unlock();
		}
		catch(...){
			LOG(WARNING) <<prefix +"error during request (connection error)";
			usleep(5000);

		}


		if ((IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()) > this->cycle ){
			DLOG(WARNING) <<prefix + ": Navdata adquisition timeout-";
		}
		else{
			usleep(this->cycle - (IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		}
		this->refreshRate=(int)(1000000/(IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		last=IceUtil::Time::now();
	}
}

JdeRobotTypes::NavdataData  NavdataIceClient::getNavdataData(){
	JdeRobotTypes::NavdataData navdata;
	this->controlMutex.lock();
	navdata = this->navdataData;
	this->controlMutex.unlock();
	return navdata;
}

} /* NS */