#include <jderobot/comm/motorsClient.hpp>
#include <jderobot/comm/ice/motorsIceClient.hpp>
#ifdef JDERROS
#include <jderobot/comm/ros/publisherMotors.hpp>
#endif

namespace JdeRobotComm {

MotorsClient* 
getMotorsClient(Ice::CommunicatorPtr ic, std::string prefix){
	MotorsClient* client = 0;
	Ice::PropertiesPtr prop = ic->getProperties();


	int server = prop->getPropertyAsIntWithDefault(prefix+".Server",0);
	switch (server){
		case 0:
		{
			std::cout << "Motors disabled" << std::endl;
			break;
		}
		case 1:
		{
			std::cout << "Sending Velocities by ICE interfaces" << std::endl;
			MotorsIceClient* cl;
			cl = new MotorsIceClient(ic, prefix);
		    client = (JdeRobotComm::MotorsClient*) cl;
		    break;
		}
		case 2:
		{
			#ifdef JDERROS
				std::cout << "Sending Velocities by ROS messages" << std::endl;
				std::string nodeName;
				nodeName =  prop->getPropertyWithDefault(prefix+".Name","MotorsNode");
				std::string topic;
				topic = prop->getPropertyWithDefault(prefix+".Topic","");
				PublisherMotors* pm;
				pm = new PublisherMotors(0, nullptr, nodeName, topic);
				pm->start();
				client = (JdeRobotComm::MotorsClient*) pm;
            #else
				throw "ERROR: ROS is not available";
			#endif
		 	break;
		}
		default:
		{
			std::cerr << "Wrong " + prefix+".Server property" << std::endl;
			break;
		}

	}

	return client;


}

}//NS
