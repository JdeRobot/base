#include <jderobot/comm/interfaces/bumperClient.hpp>
#include <jderobot/comm/ice/bumperIceClient.hpp>
#ifdef JDERROS
#include <jderobot/comm/ros/listenerBumper.hpp>
#endif

namespace Comm {

BumperClient*
getBumperClient(Comm::Communicator* jdrc, std::string prefix){
	BumperClient* client = 0;

	int server;
	std::string server_name = jdrc->getConfig().asString(prefix+".Server");
	std::transform(server_name.begin(), server_name.end(), server_name.begin(), ::tolower);
	if(server_name == "ice"){
		server = 1;
	}
	else if(server_name == "ros"){
		server = 2;
	}
	else server = 0;

	switch (server){
		case 0:
		{
			std::cout << "Bumper disabled" << std::endl;
			break;
		}
		case 1:
		{
			std::cout << "Receiving BumperData from ICE interfaces" << std::endl;
			BumperIceClient* cl;
			cl = new BumperIceClient(jdrc, prefix);
			cl->start();
		    client = (Comm::BumperClient*) cl;
		    break;
		}
		case 2:
		{
            #ifdef JDERROS
                std::cout << "Receiving BumperData from ROS messages" << std::endl;
                std::string nodeName;
                nodeName =  jdrc->getConfig().asStringWithDefault(prefix+".Name", "BumperNode");
				std::string topic;
				topic = jdrc->getConfig().asStringWithDefault(prefix+".Topic", "");
                ListenerBumper* lc;
                lc = new ListenerBumper(0, nullptr, nodeName, topic);
                lc->start();
                client = (Comm::BumperClient*) lc;
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
