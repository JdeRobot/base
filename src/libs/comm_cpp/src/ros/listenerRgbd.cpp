#include <jderobot/comm/ros/listenerRgbd.hpp>

namespace Comm {

	ListenerRgbd::ListenerRgbd(int argc, char** argv, std::string nodeName, std::string topicrgb, std::string topicd){
		pthread_mutex_init(&mutex, NULL);
		if ("" == topicrgb || "" == topicd ){
			this->on = false;
			std::cerr <<"Invalid camera topic" <<std::endl;
		}else{
			this->on = true;
			this->topicrgb = topicrgb;
			this->topicd = topicd;
			this->nodeName = nodeName;

			const std::string name = std::string(this->nodeName);

			time(&timer);
			int a = 0;
			ros::init(a, nullptr, name);
			ros::NodeHandle nh;
			int q = 1; //queue size
			this->subrgb = new message_filters::Subscriber<sensor_msgs::Image> (nh, topicrgb, q);
			this->subd = new message_filters::Subscriber<sensor_msgs::Image> (nh, topicd, q);
			this->no_cloud_sync_ = new message_filters::Synchronizer<Comm::MySyncPolicy>(Comm::MySyncPolicy(q),  *subrgb, *subd);


			this->no_cloud_sync_->registerCallback(boost::bind(&ListenerRgbd::callback, this, _1, _2));
			std::cout << "listen from "+ this->topicrgb << " and " << this->topicd << std::endl;

			this->spinner = new ros::AsyncSpinner(1);
		}
	}



	ListenerRgbd::~ListenerRgbd(){
		this->stop();
	}

	void 
	ListenerRgbd::start(){
		this->spinner->start();
	}

	void 
	ListenerRgbd::stop(){
		this->spinner->stop();
		ros::shutdown();
	}

	void 
	ListenerRgbd::callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& d){
		this->cont++;
		time_t now;
		time(&now);
		pthread_mutex_lock(&mutex);
		this->rgbd = Comm::translate_rgbd(rgb, d);
		if (difftime(this->timer, now)>=1){
			this->refreshRate = this->cont;
			this->cont = 0;
			this->timer = now;
		}
		pthread_mutex_unlock(&mutex);

	}

	JdeRobotTypes::Rgbd  ListenerRgbd::getRgbd(){
		JdeRobotTypes::Rgbd img;
		pthread_mutex_lock(&mutex);
		img = this->rgbd;
		pthread_mutex_unlock(&mutex);
		return img;
	}

	int ListenerRgbd::getRefreshRate(){

		int rr;
		pthread_mutex_lock(&mutex);
		rr = this->refreshRate;
		pthread_mutex_unlock(&mutex);

		return rr;
	}



}//NS