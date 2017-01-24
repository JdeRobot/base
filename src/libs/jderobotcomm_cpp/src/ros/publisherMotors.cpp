#include <jderobot/comm/ros/publisherMotors.hpp>

namespace JdeRobotComm {

	PublisherMotors::PublisherMotors(int argc, char** argv, std::string nodeName, std::string topic){
		pthread_mutex_init(&mutex, NULL);
		if ("" == topic){
			this->on = false;
			std::cerr <<"Invalid Motors topic" <<std::endl;
		}else{
			this->on = true;
			this->topic = topic;
			this->nodeName = nodeName;
			//boost::thread t3(&PublisherMotors::listen, this);

			const std::string name = std::string(this->nodeName);
			int a = 0;
			ros::init(a, nullptr, name);
			ros::NodeHandle nh;
			this->pub = nh.advertise<geometry_msgs::Twist>(this->topic, 1, true);
			std::cout << "publishing in "+ this->topic << std::endl;

			timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&PublisherMotors::publish, this));

			this->spinner = new ros::AsyncSpinner(1);
			
		}
	}


	void PublisherMotors::publish()
	{
		geometry_msgs::Twist vel;
		JdeRobotTypes::CMDVel cmdvel;
		pthread_mutex_lock(&mutex);
			cmdvel = this->cmdvel;
		pthread_mutex_unlock(&mutex);
		vel = translate_twist_messages (cmdvel);
		this->pub.publish(vel);


	}



	PublisherMotors::~PublisherMotors(){
		this->stop();
	}

	void 
	PublisherMotors::start(){
		this->spinner->start();
	}

	void 
	PublisherMotors::stop(){
		this->spinner->stop();
		ros::shutdown();
	}

	void
	PublisherMotors::sendVelocities(JdeRobotTypes::CMDVel vel){
		pthread_mutex_lock(&mutex);
			this->cmdvel = vel;
		pthread_mutex_unlock(&mutex);
	}

	void
	PublisherMotors::sendVX (float vx){
		pthread_mutex_lock(&mutex);
			this->cmdvel.vx = vx;
		pthread_mutex_unlock(&mutex);
	}

	void
	PublisherMotors::sendVY (float vy){
		pthread_mutex_lock(&mutex);
			this->cmdvel.vy = vy;
		pthread_mutex_unlock(&mutex);

	}

	void
	PublisherMotors::sendAZ (float az){
		pthread_mutex_lock(&mutex);
			this->cmdvel.az = az;
		pthread_mutex_unlock(&mutex);
	}

	void
	PublisherMotors::sendV (float v){
		this->sendVX(v);
	}

	void
	PublisherMotors::sendW (float w){
		this->sendAZ(w);
	}

	void
	PublisherMotors::sendL (float l ){
		this->sendVY(l);
	}



}//NS