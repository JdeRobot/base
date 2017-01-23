#include <jderobot/comm/motorsClient.hpp>

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
		 	std::cout << "Sending Velocities by ROS messages" << std::endl;
		 	/*std::string nodeName;
		 	nodeName =  prop->getPropertyWithDefault(prefix+".Name","MotorsNode");
		 	std::string topic;
		 	topic = prop->getPropertyWithDefault(prefix+".Topic","");
		 	ListenerLaser* lc;
		 	lc = new ListenerLaser(0, nullptr, nodeName, topic);
		 	lc->start();
		 	client = (JdeRobotComm::MotorsClient*) lc;
*/
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
