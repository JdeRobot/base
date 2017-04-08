/*
 *  Copyright (C) 1997-2017 JDE Developers
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
 *  Author : Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#include "jderobot/comm/ice/pose3dIceClient.hpp"

namespace JdeRobotComm {

Pose3dIceClient::Pose3dIceClient(Ice::CommunicatorPtr ic, std::string prefix) {

	this->prefix=prefix;
	Ice::PropertiesPtr prop;
	prop = ic->getProperties();

	int fps=prop->getPropertyAsIntWithDefault(prefix+".Fps",30);
	this->cycle=(float)(1/(float)fps)*1000000;


	Ice::ObjectPrx basePose = ic->propertyToProxy(prefix+".Proxy");

	if (0==basePose){
		this->on = false;
		std::cout << prefix + ".Proxy configuration not specified" <<std::endl;

	}
	else {

		try{
			this->prx = jderobot::Pose3DPrx::checkedCast(basePose);

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

Pose3dIceClient::~Pose3dIceClient() {
	this->on=false;
}


void Pose3dIceClient::pause(){
	this->controlMutex.lock();
		this->pauseStatus=true;
	this->controlMutex.unlock();
}

void Pose3dIceClient::resume(){
	this->controlMutex.lock();
		this->pauseStatus=false;
		this->sem.broadcast();
	this->controlMutex.unlock();
}

void Pose3dIceClient::run(){
	JdeRobotTypes::Pose3d pose3d;

	IceUtil::Time last;

	last=IceUtil::Time::now();
	while (this->on){
		if (pauseStatus){
			IceUtil::Mutex::Lock sync(this->controlMutex);
			this->sem.wait(sync);
		}

		try{
			jderobot::Pose3DDataPtr pose3ddata = this->prx->getPose3DData();

			pose3d.x = pose3ddata->x;
			pose3d.y = pose3ddata->y;
			pose3d.z = pose3ddata->z;
			pose3d.q[0] = pose3ddata->q0;
			pose3d.q[1] = pose3ddata->q1;
			pose3d.q[2] = pose3ddata->q2;
			pose3d.q[3] = pose3ddata->q3;
			pose3d.yaw = this->quat2Yaw(pose3d.q);
			pose3d.pitch = this->quat2Pitch(pose3d.q);
			pose3d.roll = this->quat2Roll(pose3d.q);



			this->controlMutex.lock();
			this->pose = pose3d;
			this->controlMutex.unlock();
		}
		catch(...){
			std::cerr << prefix +"error during request (connection error)" << std::endl;
			usleep(5000);

		}


		if ((IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()) <= this->cycle ){
			usleep(this->cycle - (IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		}
		last=IceUtil::Time::now();
	}
}

JdeRobotTypes::Pose3d  Pose3dIceClient::getPose(){
	JdeRobotTypes::Pose3d data;
	this->controlMutex.lock();
	data = this->pose;
	this->controlMutex.unlock();
	return data;
}


float Pose3dIceClient::quat2Yaw(std::vector <float> q){
    float rotateZa0=2.0*(q[1]*q[2] + q[0]*q[3]);
    float rotateZa1=q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    float rotateZ=0.0;
    if(rotateZa0 != 0.0 && rotateZa1 != 0.0){
        rotateZ=atan2(rotateZa0,rotateZa1);
    }
    return rotateZ;
}

float Pose3dIceClient::quat2Pitch(std::vector <float> q){
    float rotateYa0=-2.0*(q[1]*q[3] - q[0]*q[2]);
    float rotateY=0.0;
    if(rotateYa0 >= 1.0){
        rotateY = M_PI_2; // PI/2
    }
    else if (rotateYa0 <= -1.0){
        rotateY = -M_PI_2; // -PI/2
    }
    else{
        rotateY = asin(rotateYa0);
    }

    return rotateY;
}

float Pose3dIceClient::quat2Roll (std::vector <float> q){
    float rotateXa0=2.0*(q[2]*q[3] + q[0]*q[1]);
    float rotateXa1=q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    float rotateX=0.0;

    if(rotateXa0 != 0.0 && rotateXa1 != 0.0){
        rotateX=atan2(rotateXa0, rotateXa1);
    }
    return rotateX;
}

} /* NS */