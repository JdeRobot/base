/*
 *  Copyright (C) 1997-2012 JDE Developers Teameldercare.camRGB
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
 *  Authors : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */

/** \file openniServer.cpp
 * \brief openniServer component main file
 */

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/kinectleds.h>
#include <jderobot/camera.h>
#include <jderobot/pose3dmotors.h>
#include <jderobot/pointcloud.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <tr1/memory>
#include <list>
#include <sstream>
#include <math.h>
#include <cv.h>
#include <highgui.h>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "OpenNiServerLib/myprogeo.h"
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <logger/Logger.h>
#include <ns/ns.h>
#include <OpenNiServerLib/RGBCamera.h>
#include <OpenNiServerLib/DepthCamera.h>
#include <OpenNiServerLib/PointCloudServer.h>


#include "easyiceconfig/EasyIce.h"
#include "OpenNiServerLib/ConcurrentDevice.h"
#include "OpenNITypes.h"



Ice::CommunicatorPtr ic;
bool killed;
openniServer::RGBCamera *camRGB;
openniServer::DepthCamera *camDEPTH;
openniServer::PointCloudServer *pc1;
jderobot::ns* namingService = NULL;
ConcurrentDevicePtr device;


int main(int argc, char** argv){

	Ice::PropertiesPtr prop;


	try{
		ic = EasyIce::initialize(argc,argv);
		prop = ic->getProperties();
	}
	catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		return 1;
	}
	catch (const char* msg) {
		std::cerr <<"Error :" << msg << std::endl;
		return 1;
	}
	std::string componentPrefix("openniServer");


	// Analyze LOG section

	std::string logFile = prop->getProperty(componentPrefix + ".Log.File.Name");
	if (logFile.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set log file!");
	else
		jderobot::Logger::getInstance()->setFileLog(logFile);

	std::string logLevel = prop->getProperty(componentPrefix + ".Log.File.Level");
	if (logLevel.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set *.Log.File.Level key!");
	else
		jderobot::Logger::getInstance()->setFileLevel(jderobot::Levels(boost::lexical_cast<int>(logLevel)));

	std::string screenLevel = prop->getProperty(componentPrefix + ".Log.Screen.Level");
	if (screenLevel.size()==0)
		jderobot::Logger::getInstance()->warning("You didn't set *.Log.Screen.Level key!");
	else
		jderobot::Logger::getInstance()->setScreenLevel(jderobot::Levels(boost::lexical_cast<int>(screenLevel)));

	jderobot::Logger::getInstance()->info("Logger:: screenLevel=" + screenLevel + " logLevel=" + logLevel + " LogFile=" + logFile);



	bool cameraR = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraRGB",0);
	bool cameraD = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraDEPTH",0);
	int imageRegistration = prop->getPropertyAsIntWithDefault(componentPrefix + ".ImageRegistration",1);
	int motors = prop->getPropertyAsIntWithDefault(componentPrefix + ".Pose3DMotorsActive",0);
	int leds = prop->getPropertyAsIntWithDefault(componentPrefix + ".KinectLedsActive",0);
	int pointCloud = prop->getPropertyAsIntWithDefault(componentPrefix + ".pointCloudActive",0);

    //todo
//	mirrorDepth = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraDEPTH.Mirror",0);
//	mirrorRGB = prop->getPropertyAsIntWithDefault(componentPrefix + ".CameraRGB.Mirror",0);
	int deviceMode=prop->getPropertyAsIntWithDefault(componentPrefix + ".Mode", 0);
	int deviceFPS=prop->getPropertyAsIntWithDefault(componentPrefix + ".Hz", 20);
	std::string Endpoints = prop->getProperty(componentPrefix + ".Endpoints");
	Ice::ObjectAdapterPtr adapter =ic->createObjectAdapterWithEndpoints(componentPrefix, Endpoints);

	int cameraID = prop->getPropertyAsIntWithDefault(componentPrefix + ".deviceId",0);
	jderobot::Logger::getInstance()->info( "OpenniServer: Selected device: " + cameraID );
	int nCameras=0;

    DeviceConfig config;
    if (cameraR){
        config.openColor=SENSOR_ON;
    }
    if (cameraD){
        config.openDepth=SENSOR_ON;
    }
    config.registrationMode=RegistrationMode(imageRegistration);


    device= ConcurrentDevicePtr(new ConcurrentDevice(deviceFPS,cameraID,config,deviceMode));
    device->start();


	// Naming Service
	int nsActive = prop->getPropertyAsIntWithDefault("NamingService.Enabled", 0);

	if (nsActive)
	{
		std::string ns_proxy = prop->getProperty("NamingService.Proxy");
		try
		{
			namingService = new jderobot::ns(ic, ns_proxy);
		}
		catch (Ice::ConnectionRefusedException& ex)
		{
			jderobot::Logger::getInstance()->error("Impossible to connect with NameService!");
			exit(-1);
		}
	}

	if (cameraR){
		std::string objPrefix(componentPrefix + ".CameraRGB.");
		std::string cameraName = prop->getProperty(objPrefix + "Name");
		if (cameraName.size() == 0){//no name specified, we create one using the index
			cameraName = "cameraR";
			prop->setProperty(objPrefix + "Name",cameraName);//set the value
		}
		jderobot::Logger::getInstance()->info("Creating camera " + cameraName );
		camRGB = new openniServer::RGBCamera(objPrefix,ic,device);
		adapter->add(camRGB, ic->stringToIdentity(cameraName));
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: CameraRGB created successfully(" + Endpoints + "@" + cameraName );


		if (namingService)
			namingService->bind(cameraName, Endpoints, camRGB->ice_staticId());



	}

	if (cameraD){
		std::string objPrefix(componentPrefix + ".CameraDEPTH.");
		std::string cameraName = prop->getProperty(objPrefix + "Name");
		if (cameraName.size() == 0){//no name specified, we create one using the index
			cameraName = "cameraD";
			prop->setProperty(objPrefix + "Name",cameraName);//set the value
		}
		jderobot::Logger::getInstance()->info( "Creating camera " +  cameraName );
		camDEPTH = new openniServer::DepthCamera(objPrefix,ic,device);
		adapter->add(camDEPTH, ic->stringToIdentity(cameraName));
		//test camera ok
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: CameraDEPTH created successfully(" + Endpoints + "@" + cameraName );

		if (namingService)
			namingService->bind(cameraName, Endpoints, camDEPTH->ice_staticId());

	}

	if (pointCloud){
		std::string objPrefix(componentPrefix + ".PointCloud.");
		std::string Name = prop->getProperty(objPrefix + "Name");
		jderobot::Logger::getInstance()->info( "Creating pointcloud1 " + Name );
		pc1 = new openniServer::PointCloudServer(objPrefix,prop,device);
		adapter->add(pc1 , ic->stringToIdentity(Name));
		jderobot::Logger::getInstance()->info("              -------- openniServer: Component: PointCloud created successfully(" + Endpoints + "@" + Name );
	}
	adapter->activate();
	ic->waitForShutdown();
	adapter->destroy();

	return 0;

}
