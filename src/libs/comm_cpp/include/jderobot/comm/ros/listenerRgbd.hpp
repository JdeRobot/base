/*
 *  Copyright (C) 1997-2016 JDE Developers Team
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

#ifndef JDEROBOTCOMM_LISTENERCAMERA_H_
#define JDEROBOTCOMM_LISTENERCAMERA_H_


#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <boost/thread/thread.hpp>
#include <jderobot/types/rgbd.h>
#include <jderobot/comm/interfaces/rgbdClient.hpp>
#include <jderobot/comm/ros/translators.hpp>
#include <boost/bind.hpp>
#include <time.h>

namespace Comm {

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image> MySyncPolicy;
	
	class ListenerRgbd: public Comm::RgbdClient {
		
	public:
		ListenerRgbd(int argc, char** argv, std::string nodeName, std::string topicrgb, std::string topicd);
		~ListenerRgbd();

		void start();
		void stop();
		virtual JdeRobotTypes::Rgbd getRgbd();
		virtual int getRefreshRate();
		

	private:
		pthread_mutex_t mutex;
		message_filters::Subscriber<sensor_msgs::Image>* subrgb;      
		message_filters::Subscriber<sensor_msgs::Image>* subd;           
		message_filters::Synchronizer<Comm::MySyncPolicy>* no_cloud_sync_;
		std::string topicrgb;
		std::string topicd;
		std::string nodeName;

		int cont = 0; //used to count Frames per seconds
		time_t timer; // used to save time for FPS

		ros::AsyncSpinner* spinner;
		
		
		void callback (const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& d);

		



	};//class

} //NS
#endif /* JDEROBOTCOMM_LISTENERCAMERA_H_ */