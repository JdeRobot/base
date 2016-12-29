#include "jderobot/com/ros/translators.hpp"
namespace JdeRobotCom {

	float PI = 3.1415;

	JdeRobotTypes::LaserData 
	translate_laser_messages(const sensor_msgs::LaserScanConstPtr& scan)
	{
		JdeRobotTypes::LaserData data;

		data.values = scan->ranges;
		data.minRange = scan->range_min;
		data.maxRange = scan->range_max;
		data.maxAngle = scan->angle_max  - PI/2;
		data.minAngle = scan->angle_min - PI/2;
		//falta timestamp

		return data;

	}




} /* NS */