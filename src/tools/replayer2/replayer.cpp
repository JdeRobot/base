/*
 *
 *  Copyright (C) 1997-2013 JDERobot Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors :
 *						Francisco Rivas <franciscomiguel.rivas@urjc.es>
 *
 */


#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <tr1/memory>
#include <list>
#include <sstream>
#include <iostream>
#include <fstream>
// Library includes
#include <cv.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <jderobot/camera.h>
#include <jderobot/pointcloud.h>
#include <jderobot/pose3dencoders.h>
#include <jderobot/pose3d.h>
#include <jderobot/encoders.h>
#include <jderobot/replayControl.h>
#include <jderobot/laser.h>
#include "replayergui.h"
#include <ns/ns.h>
#include <interfaces/ReplayerCamera.h>
#include <interfaces/ReplayerPointCloud.h>
#include <interfaces/ReplayerLaser.h>
#include <interfaces/ReplayerPose3DEncoders.h>
#include <interfaces/ReplayerPose3D.h>
#include <interfaces/ReplayerEncoders.h>
#include <utils/ReplayControllerInterface.h>

#include "easyiceconfig/EasyIce.h" 


bool componentAlive;
bool killed;
Ice::CommunicatorPtr ic;
jderobot::nsPtr ns;
void exitApplication(int s){


	killed=true;
	componentAlive=false;

	// NamingService
	if (ns)
	{
		ns->unbindAll();
	}

	ic->shutdown();
	exit(0);

}


int main(int argc, char** argv) {

	std::vector<Ice::ObjectPtr> cameras;
	IceUtil::Time a = IceUtil::Time::now();
	long long int initState=(a.toMicroSeconds())/1000;
	int nProcs=0;
	std::string componentPrefix("Replayer");


	componentAlive=true;
	killed=false;
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = exitApplication;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

    replayer::SyncControllerPtr syncControllerPtr;

	try{
		ic = EasyIce::initialize(argc, argv);
		Ice::PropertiesPtr prop = ic->getProperties();

		bool startPlaying = (bool)prop->getPropertyAsIntWithDefault(componentPrefix+".startPlaying",1);
		bool repeat = (bool)prop->getPropertyAsIntWithDefault(componentPrefix+".repeat",1);

//		replayer::controller= new replayer::SyncController(initState,startPlaying,repeat);
        syncControllerPtr=replayer::SyncControllerPtr(new replayer::SyncController(initState,startPlaying,repeat));
		// Naming Service
		int nsActive = prop->getPropertyAsIntWithDefault("NamingService.Enabled", 0);
		if (nsActive)
			ns = jderobot::nsPtr(new jderobot::ns(ic,"NamingService.Proxy",nsActive));
		if (nsActive && !ns->getIceProxy()){
			return -1;
		}

		// Analyze LOG section
		jderobot::Logger::initialize(argv[0],prop,componentPrefix);

		int nCameras = prop->getPropertyAsIntWithDefault(componentPrefix+".nCameras",0);
		LOG(INFO) << "Cameras to load: " + boost::lexical_cast<std::string>(nCameras);
		std::string Endpoints = prop->getProperty(componentPrefix+".Endpoints");
		cameras.resize(nCameras);
		Ice::ObjectAdapterPtr adapter =ic->createObjectAdapterWithEndpoints(componentPrefix+"", Endpoints);
		for (int i=0; i<nCameras; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".Camera." + objId + ".");
			std::string cameraName = prop->getProperty(objPrefix + "Name");
			LOG(INFO) << "Creating camera: " + cameraName;

			if (cameraName.size() == 0) { //no name specified, we create one using the index
				cameraName = "camera" + objId;
				prop->setProperty(objPrefix + "Name",cameraName);//set the value
			}
			cameras[i] = new replayer::ReplayerCamera(objPrefix,ic,syncControllerPtr,initState);
			adapter->add(cameras[i], ic->stringToIdentity(cameraName));
			if (ns)
				ns->bind(cameraName, Endpoints, "::jderobot::Camera");
			nProcs++;
		}
		int nPointClouds = prop->getPropertyAsIntWithDefault(componentPrefix+".nPointClouds",0);
		for (int i=0; i<nPointClouds; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".PointCloud." + objId + ".");
			std::string Name = prop->getProperty(objPrefix + "Name");
			LOG(INFO) <<"Creating pointCloud " + Name ;


			if (Name.size() == 0) { //no name specified, we create one using the index
				Name = "pointcloud" + objId;
				prop->setProperty(objPrefix + "Name",Name);//set the value
			}




			Ice::ObjectPtr  object= new replayer::ReplayerPointCloud(objPrefix,ic,syncControllerPtr,initState);



			adapter->add(object, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, object->ice_staticId());
			nProcs++;
		}

		int nLasers = prop->getPropertyAsIntWithDefault(componentPrefix+".nLasers",0);
		for (int i=0; i<nLasers; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".laser." + objId + ".");
			std::string Name = prop->getProperty(objPrefix + "Name");
			LOG(INFO) <<"Creating laser " + Name ;

			if (Name.size() == 0) { //no name specified, we create one using the index
				Name = "laser" + objId;
				prop->setProperty(objPrefix + "Name",Name);//set the value
			}
			Ice::ObjectPtr  object= new replayer::ReplayerLaser(objPrefix,ic,syncControllerPtr,initState);
			adapter->add(object, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, object->ice_staticId());
			nProcs++;
		}



		int nPose3dEncoders = prop->getPropertyAsIntWithDefault(componentPrefix+".nPose3dEncoders",0);
		for (int i=0; i<nPose3dEncoders; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".pose3dencoder." + objId + ".");
			std::string Name = prop->getProperty(objPrefix + "Name");
			LOG(INFO) <<"Creating pose3dencoders " + Name ;

			if (Name.size() == 0) { //no name specified, we create one using the index
				Name = "pose3dencoders" + objId;
				prop->setProperty(objPrefix + "Name",Name);//set the value
			}

			Ice::ObjectPtr  object= new replayer::ReplayerPose3DEncoders(objPrefix,ic,syncControllerPtr,initState);



			adapter->add(object, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, object->ice_staticId());
			nProcs++;
		}


		int nPose3d = prop->getPropertyAsIntWithDefault(componentPrefix+".nPose3d",0);
		for (int i=0; i<nPose3d; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".pose3d." + objId + ".");
			std::string Name = prop->getProperty(objPrefix + "Name");
            LOG(INFO) <<"Creating pose3d " + Name ;

			if (Name.size() == 0) { //no name specified, we create one using the index
				Name = "pose3d" + objId;
				prop->setProperty(objPrefix + "Name",Name);//set the value
			}

			Ice::ObjectPtr  object= new replayer::ReplayerPose3D(objPrefix,ic,syncControllerPtr,initState);



			adapter->add(object, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, object->ice_staticId());
			nProcs++;
		}

		int nEncoders = prop->getPropertyAsIntWithDefault(componentPrefix+".nEncoders",0);
		for (int i=0; i<nEncoders; i++) {//build camera objects
			std::stringstream objIdS;
			objIdS <<  i;
			std::string objId = objIdS.str();
			std::string objPrefix(componentPrefix+".encoder." + objId + ".");
			std::string Name = prop->getProperty(objPrefix + "Name");
			LOG(INFO) <<"Creating encoders " + Name ;


			if (Name.size() == 0) { //no name specified, we create one using the index
				Name = "encoders" + objId;
				prop->setProperty(objPrefix + "Name",Name);//set the value
			}

			Ice::ObjectPtr  object= new replayer::ReplayerEncoders(objPrefix,ic,syncControllerPtr,initState);



			adapter->add(object, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, object->ice_staticId());
			nProcs++;
		}


		int controllerActive = prop->getPropertyAsIntWithDefault(componentPrefix+".replayControl.Active",0);
		if (controllerActive){
			Ice::ObjectPtr rc= new replayer::ReplayControllerInterface(syncControllerPtr);
			std::string objPrefix(componentPrefix+".replayControl.");
			std::string Name = prop->getProperty(objPrefix + "Name");

			adapter->add(rc, ic->stringToIdentity(Name));
			if (ns)
				ns->bind(Name, Endpoints, rc->ice_staticId());
		}

		adapter->activate();
        syncControllerPtr->setProcesses(nProcs);

		//replayer::replayergui* gui = new replayer::replayergui(replayer::controller);


		while (true){
			//gui->update();
			syncControllerPtr->checkStatus();
			usleep(1000000);
		}



		ic->waitForShutdown();
	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		exit(-1);
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		exit(-1);
	}
	return 0;
}
