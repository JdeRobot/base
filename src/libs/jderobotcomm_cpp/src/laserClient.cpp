#include <jderobot/comm/interfaces/laserClient.hpp>
#include <jderobot/comm/ice/laserIceClient.hpp>
#ifdef JDERROS
#include <jderobot/comm/ros/listenerLaser.hpp>
#endif

namespace JdeRobotComm {

LaserClient* 
getLaserClient(JdeRobotComm::Communicator* jdrc, std::string prefix){
	LaserClient* client = 0;

	int server = jdrc->getConfig().asIntWithDefault(prefix+".Server", 0);
	switch (server){
		case 0:
		{
			std::cout << "Laser disabled" << std::endl;
			break;
		}
		case 1:
		{
			std::cout << "Receiving LaserData from ICE interfaces" << std::endl;
			LaserIceClient* cl;
			cl = new LaserIceClient(jdrc, prefix);
			cl->start();
		    client = (JdeRobotComm::LaserClient*) cl;
		    break;
		}
		case 2:
		{
            #ifdef JDERROS
                std::cout << "Receiving LaserData from ROS messages" << std::endl;
                std::string nodeName;
                nodeName =  jdrc->getConfig().asStringWithDefault(prefix+".Name", "LaserNode");
				std::string topic;
				topic = jdrc->getConfig().asStringWithDefault(prefix+".Topic", "");
                ListenerLaser* lc;
                lc = new ListenerLaser(0, nullptr, nodeName, topic);
                lc->start();
                client = (JdeRobotComm::LaserClient*) lc;
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
