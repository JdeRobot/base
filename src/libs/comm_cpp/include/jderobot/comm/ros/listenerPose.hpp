/*
 *  Copyright (C) 1997-2017 JDE Developers Team
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
 *       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#ifndef JDEROBOTCOMM_LISTENERPOSE3D_H_
#define JDEROBOTCOMM_LISTENERPOSE3D_H_


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/thread.hpp>
#include <jderobot/types/pose3d.h>
#include <jderobot/comm/interfaces/pose3dClient.hpp>
#include <jderobot/comm/ros/translators.hpp>

namespace Comm {
	class ListenerPose: public Comm::Pose3dClient {
		
	public:
		ListenerPose(int argc, char** argv, std::string nodeName, std::string topic);
		~ListenerPose();

		void start();
		void stop();
		virtual JdeRobotTypes::Pose3d  getPose();
		

	private:
		pthread_mutex_t mutex;
		ros::Subscriber sub;
		std::string topic;
		std::string nodeName;

		ros::AsyncSpinner* spinner;
		
		void posecallback (const nav_msgs::OdometryConstPtr& odom_msg);

		



	};//class

} //NS
#endif /* JDEROBOTCOMM_LISTENERPOSE3D_H_ */