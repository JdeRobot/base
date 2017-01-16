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

#ifndef JDEROBOTCOMM_POSE3DICECLIENT_H_
#define JDEROBOTCOMM_POSE3DICECLIENT_H_

#include <IceUtil/IceUtil.h>
#include <iostream>
#include <Ice/Ice.h>
#include <jderobot/pose3d.h>
#define _USE_MATH_DEFINES
#include <math.h> 
#include <jderobot/types/pose3d.h>
#include <jderobot/comm/interfaces/pose3dClient.hpp>

namespace JdeRobotComm {


class Pose3dIceClient: public IceUtil::Thread, public JdeRobotComm::Pose3dClient {
public:
	Pose3dIceClient(Ice::CommunicatorPtr ic, std::string prefix);
	virtual ~Pose3dIceClient();
	virtual void run();

	virtual JdeRobotTypes::Pose3d  getPose3d();
	void pause();
	void resume();
	bool getPause(){return pauseStatus;};



private:
	std::string prefix;
	jderobot::Pose3DPrx prx;
	long long int cycle;
	IceUtil::Mutex controlMutex;
	bool pauseStatus;

	IceUtil::Cond sem;

	float quat2Yaw(vector <float> q);

	float quat2Pitch(vector <float> q);

	float quat2Roll(vector <float> q);

};


} /* namespace jderobotcomm */
#endif /* JDEROBOTCOMM_POSE3DICECLIENT_H_ */