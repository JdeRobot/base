/*
 *  Copyright (C) 1997-2015 JDE Developers Team
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
 *  Authors : 
 *       Alberto Martín Florido <almartinflorido@gmail.com>
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */

#ifndef _DRONE_CAMERASERVER_H_
#define _DRONE_CAMERASERVER_H_

#include "../ardrone_sdk.h"
#include "../ardrone_driver.h"
#include <Ice/Ice.h>
#include <jderobot/pose3d.h>

namespace pose3D
{
	typedef struct pose3d{
		union{double x; double roll;};
		union{double y; double pitch;};
		union{double z; double yaw;};
	}pose3d_t;
	#define Point3D_OP(c,a,op,b) {c.x = a.x op b.x; c.y = a.y op b.y; c.z = a.z op b.z;}


	class Pose3DI : virtual public jderobot::Pose3D
	{
		public:
			Pose3DI(ARDroneDriver *driver);
			virtual ~Pose3DI();			
			virtual jderobot::Pose3DDataPtr getPose3DData(const Ice::Current&);
			virtual Ice::Int setPose3DData(const jderobot::Pose3DDataPtr& data, const Ice::Current&);		
		private:
			void poseBoostrapWithExternals(pose3d_t current_xyz, pose3d_t current_angles);

			jderobot::Pose3DDataPtr pose3D;
			ARDroneDriver *driver;

			bool gps_on, gps_valid;
			bool gps_is_bootstrapped;

			Eigen::Quaternion<float> qR;
			pose3d_t xyzT;
	};


}

#endif
