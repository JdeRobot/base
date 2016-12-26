#include <jderobot/com/laserClient.hpp>

namespace JdeRobotCom {

LaserClient* 
getLaserClient(Ice::CommunicatorPtr ic, std::string prefix){
	LaserClient* client = 0;
	Ice::PropertiesPtr prop = ic->getProperties();


	int server = prop->getPropertyAsIntWithDefault(prefix+".Server",0);
	switch (server){

		case 0:
			std::cout << "Receiving LaserData from ICE interfaces" << std::endl;
			LaserIceClient* cl;
			cl = new LaserIceClient(ic, prefix);
			cl->start();
		    client = (JdeRobotCom::LaserClient*) cl;
		    break;
		case 1:
		 	std::cout << "Receiving LaserData from ROS messages" << std::endl;
		 	break;
		default:
			break;

	}

	return client;


}

}//NS