/*
 *  Copyright (C) 1997-2013 JDE Developers TeamrgbdViewer.camRGB
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
 *  Author : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */





#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/pointcloud.h>
#include <rgbd.h>
#include <jderobotutil/utils/CameraUtils.h>
#include "rgbdViewergui.h"
#include "pthread.h"
#include "parallelIce/cameraClient.h"
#include "parallelIce/pointcloudClient.h"

#include "easyiceconfig/EasyIce.h" 



#define MAX_COMPONENTS 20	

rgbdViewer::rgbdViewergui* rgbdViewergui_ptx;

jderobot::CameraClientPtr camRGB;
jderobot::CameraClientPtr camDEPTH;
jderobot::PointcloudClientPtr pcClient;
jderobot::rgbdPrx rgbClient;
bool cameraRGBDActive=false;


int debug;



void *gui_thread(void* arg){
	try{
		//local data
		std::vector<jderobot::RGBPoint> cloud;
		cv::Mat rgb,depth;

		IceUtil::Time lastIT;

		lastIT=IceUtil::Time::now();
		while(rgbdViewergui_ptx->isVisible() && ! rgbdViewergui_ptx->isClosed()){

			if (cameraRGBDActive) {
				auto data = rgbClient->getData();
				rgb = CameraUtils::getImageFromCameraProxy(data.color);
				depth = CameraUtils::getImageFromCameraProxy(data.depth);
			}
			else {
				if (camRGB)
					camRGB->getImage(rgb);
				if (camDEPTH)
					camDEPTH->getImage(depth);
			}
			if (pcClient)
				pcClient->getData(cloud);


			if ((rgb.rows!=0)&&(depth.rows!=0)){
				rgbdViewergui_ptx->updateAll(rgb,depth, cloud);
			}
			else if (rgb.rows!=0){
				rgbdViewergui_ptx->updateRGB(rgb);
			}
			else if (depth.rows!=0){
				rgbdViewergui_ptx->updateDEPTH(depth);
			}
			else{	
				rgbdViewergui_ptx->updatePointCloud(cloud);
			}
			if (((IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds()) > rgbdViewergui_ptx->getCycle() )){
				if (debug)
					std::cout<<"-------- rgbdViewer: timeout-" << std::endl;
			}
			else{
				usleep(rgbdViewergui_ptx->getCycle() - (IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds()));
			}
			lastIT=IceUtil::Time::now();


		}
	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	}
	catch (const char* msg) {
		std::cerr << msg << std::endl;
	}
	pthread_exit(NULL);
	return NULL;
}

/**
 * \brief Main program function code
 */
int main(int argc, char** argv){

	int i;
	Ice::CommunicatorPtr ic;
	int n_components=0;
	pthread_t threads[MAX_COMPONENTS];
	pthread_attr_t attr;
	Ice::PropertiesPtr prop;
	bool create_gui=false;



	bool rgbCamSelected=false;
	bool depthCamSelected=false;
	bool pointCloudSelected=false;


	bool cameraRGBActive=false;
	bool cameraDepthActive=false;





	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	try{
		ic = EasyIce::initialize(argc,argv);
		prop = ic->getProperties();
	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		return 1;
	}
	catch (const char* msg) {
		std::cerr <<"Error :" << msg << std::endl;
		return 1;
	}
	cameraRGBActive=(bool)prop->getPropertyAsIntWithDefault("rgbdViewer.CameraRGBActive",0);
	cameraDepthActive=(bool)prop->getPropertyAsIntWithDefault("rgbdViewer.CameraDEPTHActive",0);
	cameraRGBDActive=(bool)prop->getPropertyAsIntWithDefault("rgbdViewer.RGBDActive",0);

	if (cameraRGBDActive && (cameraRGBActive ||  cameraDepthActive)){
		LOG(ERROR) << "RGBD and single cameras cannot be selected at the same time";
		return 2;
	}


	if (cameraRGBDActive) {
		std::string prefix = "rgbdViewer.RGBD.";
		try{
			auto base = ic->propertyToProxy(prefix+"Proxy");
			if (0==base){
				throw prefix + "Could not create proxy with RGBD";
			}
			else {
				rgbClient = jderobot::rgbdPrx::checkedCast(base);
				if (0==rgbClient)
					throw "Invalid " + prefix + ".Proxy";
			}
		}catch (const Ice::Exception& ex) {
			std::cerr << ex << std::endl;
		}
		catch (const char* msg) {
			std::cerr << msg << std::endl;
			LOG(FATAL) << prefix + " Not camera provided";
		}
		create_gui = true;
		rgbCamSelected=true;
		depthCamSelected=true;
	}
	else{
		if (cameraRGBActive) {
			camRGB = jderobot::CameraClientPtr(new jderobot::cameraClient(ic, "rgbdViewer.CameraRGB."));
			if (camRGB != NULL) {
				rgbCamSelected = true;
				camRGB->start();
				create_gui = true;
			} else {
				throw "rgbdViewer: failed to load RGB Camera";
			}

		}
		if (cameraDepthActive) {
			camDEPTH =jderobot::CameraClientPtr( new jderobot::cameraClient(ic, "rgbdViewer.CameraDEPTH."));
			if (camDEPTH != NULL) {
				depthCamSelected = true;
				camDEPTH->start();
				create_gui = true;
			} else {
				throw "rgbdViewer: failed to load DEPTH Camera";
			}
		}
	}


	if (prop->getPropertyAsIntWithDefault("rgbdViewer.pointCloudActive",0)){
		pcClient = jderobot::PointcloudClientPtr(new jderobot::pointcloudClient(ic,"rgbdViewer.pointCloud."));
		if (pcClient!= NULL){
			pcClient->start();
			pointCloudSelected=true;
			create_gui=true;
		}
		else{
			throw "rgbdViewer: failed to load pointCloud";
		}
	}

	//set the sizes of each image
	cv::Size rgbSize(0,0);
	cv::Size depthSize(0,0);
	//rgb
	if (cameraRGBDActive) {
		while ((rgbSize == cv::Size(0, 0)) && (depthSize == cv::Size(0, 0))) {
			auto data = rgbClient->getData();
			rgbSize = CameraUtils::getImageFromCameraProxy(data.color).size();
			depthSize = CameraUtils::getImageFromCameraProxy(data.depth).size();
		}
	}
	else {
		if (rgbCamSelected) {
			while (rgbSize == cv::Size(0, 0)) {
				cv::Mat temp;
				camRGB->getImage(temp, true);
				rgbSize = temp.size();

			}
		}
		//depth
		if (depthCamSelected) {
			while (depthSize == cv::Size(0, 0)) {
				cv::Mat temp;
				camDEPTH->getImage(temp, true);
				depthSize = temp.size();

			}
		}
	}


	debug=prop->getPropertyAsIntWithDefault("rgbdViewer.Debug",320);
	int fps=prop->getPropertyAsIntWithDefault("rgbdViewer.Fps",0);
	float cycle=(float)(1/(float)fps)*1000000;

	rgbdViewergui_ptx = new rgbdViewer::rgbdViewergui(rgbCamSelected,depthCamSelected, pointCloudSelected, prop->getProperty("rgbdViewer.WorldFile"), prop->getProperty("rgbdViewer.camRGB"), prop->getProperty("rgbdViewer.camIR"),rgbSize,depthSize, cycle);
	
	std::cout << create_gui << std::endl;
	if (create_gui){
		pthread_create(&threads[n_components], &attr, gui_thread,NULL);
		n_components++;
	}


	if (rgbdViewergui_ptx == NULL)
		throw "rgbdViewer: Could not create the grafic interface";
	for (i = 0; i < n_components; i++) {
		pthread_join(threads[i], NULL);
	}

	std::cout << "final" << std::endl;
	if (ic)
		ic->destroy();
	return 0;
}
