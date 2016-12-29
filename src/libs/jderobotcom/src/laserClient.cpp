#include <jderobot/com/laserClient.hpp>

namespace JdeRobotCom {

LaserClient* 
getLaserClient(Ice::CommunicatorPtr ic, std::string prefix){
	LaserClient* client = 0;
	Ice::PropertiesPtr prop = ic->getProperties();


	int server = prop->getPropertyAsIntWithDefault(prefix+".Server",0);
	switch (server){

		case 0:
		{
			std::cout << "Receiving LaserData from ICE interfaces" << std::endl;
			LaserIceClient* cl;
			cl = new LaserIceClient(ic, prefix);
			cl->start();
		    client = (JdeRobotCom::LaserClient*) cl;
		    break;
		}
		case 1:
		{
		 	std::cout << "Receiving LaserData from ROS messages" << std::endl;
		 	std::string nodeName;
		 	nodeName =  prop->getPropertyWithDefault(prefix+".Name","LaserNode");
		 	std::string topic;
		 	topic = prop->getPropertyWithDefault(prefix+".Topic","");
		 	ListenerLaser* lc;
		 	lc = new ListenerLaser(0, nullptr, nodeName, topic);
		 	//lc->listen();
		 	client = (JdeRobotCom::LaserClient*) lc;

		 	break;
		}
		default:
		{
			std::cerr << "Wrong " + prefix+".Server propertie" << std::endl;
			break;
		}

	}

	return client;


}

}//NS