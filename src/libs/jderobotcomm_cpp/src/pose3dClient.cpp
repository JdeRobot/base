#include <jderobot/comm/pose3dClient.hpp>

namespace JdeRobotComm {

Pose3dClient* 
getPose3dClient(Ice::CommunicatorPtr ic, std::string prefix){
	Pose3dClient* client = 0;
	Ice::PropertiesPtr prop = ic->getProperties();


	int server = prop->getPropertyAsIntWithDefault(prefix+".Server",0);
	switch (server){
		case 0:
		{
			std::cout << "Pose3d disabled" << std::endl;
			break;
		}
		case 1:
		{
			std::cout << "Receiving Pose3D from ICE interfaces" << std::endl;
			Pose3dIceClient* cl;
			cl = new Pose3dIceClient(ic, prefix);
			cl->start();
		    client = (JdeRobotComm::Pose3dClient*) cl;
		    break;
		}
		case 2:
		{
		 	std::cout << "Receiving Pose3D from ROS messages" << std::endl;
		 	/*std::string nodeName;
		 	nodeName =  prop->getPropertyWithDefault(prefix+".Name","LaserNode");
		 	std::string topic;
		 	topic = prop->getPropertyWithDefault(prefix+".Topic","");
		 	ListenerPose* lc;
		 	lc = new ListenerPose(0, nullptr, nodeName, topic);
		 	//lc->listen();
		 	client = (JdeRobotComm::Pose3dClient*) lc;
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