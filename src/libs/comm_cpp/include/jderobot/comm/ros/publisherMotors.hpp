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

#ifndef JDEROBOTCOMM_PUBLISHERCMDVEL_H_
#define JDEROBOTCOMM_PUBLISHERCMDVEL_H_


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <jderobot/types/cmdvel.h>
#include <jderobot/comm/interfaces/motorsClient.hpp>
#include <jderobot/comm/ros/translators.hpp>

namespace Comm {
	class PublisherMotors: public Comm::MotorsClient {
		
	public:
		PublisherMotors(int argc, char** argv, std::string nodeName, std::string topic);
		~PublisherMotors();

		void start();
		void stop();
		
		virtual void sendVelocities(JdeRobotTypes::CMDVel vel);
		virtual void sendVX (float vx);
		virtual void sendVY (float vy);
		virtual void sendAZ (float az);
		virtual void sendV (float v);
		virtual void sendW (float w);
		virtual void sendL (float l );
		

	private:
		pthread_mutex_t mutex;
		ros::Publisher pub;
		std::string topic;
		std::string nodeName;

		ros::AsyncSpinner* spinner;
		ros::Timer timer_;
		bool forPublish;

		void publish();
		

	};//class

} //NS
#endif /* JDEROBOTCOMM_PUBLISHERCMDVEL_H_ */