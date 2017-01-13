#include "jderobot/comm/ros/translators.hpp"
namespace JdeRobotComm {

	float PI = 3.1415;

	JdeRobotTypes::LaserData 
	translate_laser_messages(const sensor_msgs::LaserScanConstPtr& scan)
	{
		JdeRobotTypes::LaserData data;

		data.values = scan->ranges;
		data.minRange = scan->range_min;
		data.maxRange = scan->range_max;
		/* 
		*      ROS Angle Map      JdeRobot Angle Map
		*            0                  PI/2
		*            |                   |
		*            |                   |
		*   PI/2 --------- -PI/2  PI --------- 0
		*            |                   |
		*            |                   |
		*/
		data.maxAngle = scan->angle_max  - PI/2;
		data.minAngle = scan->angle_min - PI/2;
		//nsec --> nanoseconds
		data.timeStamp = scan->header.stamp.sec + (scan->header.stamp.nsec *1e-9);


		return data;

	}


	JdeRobotTypes::Image 
	translate_image_messages(const sensor_msgs::ImageConstPtr& image_msg){
		JdeRobotTypes::Image img;
		cv_bridge::CvImagePtr cv_ptr;

		img.timeStamp = image_msg->header.stamp.sec + (image_msg->header.stamp.nsec *1e-9);
		img.format = "RGB8"; // we convert img_msg to RGB8 format
		img.width = image_msg->width;
		img.height = image_msg->height;

		try {

			cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
		} catch (cv_bridge::Exception& e) {

			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		img.data = cv_ptr->image;

		return img;
	}




} /* NS */