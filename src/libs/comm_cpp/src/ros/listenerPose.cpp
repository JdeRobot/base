#include <jderobot/comm/ros/listenerPose.hpp>

namespace Comm {

	ListenerPose::ListenerPose(int argc, char** argv, std::string nodeName, std::string topic){
		pthread_mutex_init(&mutex, NULL);
		if ("" == topic){
			this->on = false;
			std::cerr <<"Invalid laser topic" <<std::endl;
		}else{
			this->on = true;
			this->topic = topic;
			this->nodeName = nodeName;
			//boost::thread t3(&ListenerPose::listen, this);

			const std::string name = std::string(this->nodeName);
			int a = 0;
			ros::init(a, nullptr, name);
			ros::NodeHandle nh;
			this->sub = nh.subscribe(this->topic, 1001, &ListenerPose::posecallback, this);
			std::cout << "listen from "+ this->topic << std::endl;

			this->spinner = new ros::AsyncSpinner(1);
			
		}
	}



	ListenerPose::~ListenerPose(){
		this->stop();
	}

	void 
	ListenerPose::start(){
		this->spinner->start();
	}

	void 
	ListenerPose::stop(){
		this->spinner->stop();
		ros::shutdown();
	}

	void 
	ListenerPose::posecallback(const nav_msgs::OdometryConstPtr& odom_msg){
		pthread_mutex_lock(&mutex);
		this->pose = Comm::translate_odometry_messages(odom_msg);
		pthread_mutex_unlock(&mutex);

	}

	JdeRobotTypes::Pose3d  
	ListenerPose::getPose(){
		JdeRobotTypes::Pose3d pose3d;
		pthread_mutex_lock(&mutex);
		pose3d = this->pose;
		pthread_mutex_unlock(&mutex);
		return pose3d;
	}



}//NS