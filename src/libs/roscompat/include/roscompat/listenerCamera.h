
#ifndef ROSLISTENERCAMERA_H
#define ROSLISTENERCAMERA_H

#include "ros/ros.h"
#include "roscompat/roscompat.h"

#include "Num.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

//namespace roscompat {

class listenerCamera{
	
public:

	void listen(int argc, char** argv, std::string topic);
	void stop();
	cv::Mat getNewFrame();
	int getNFrame();

private:
	pthread_mutex_t mutex;

};//class

//}//NS

#endif // ROSLISTENERCAMERA_H

	
