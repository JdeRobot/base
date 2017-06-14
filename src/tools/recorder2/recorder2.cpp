#include <iostream>
#include <fstream>
#include <pthread.h> 
#include <signal.h>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/camera.h>
#include <jderobot/motors.h>
#include <jderobot/ptmotors.h>
#include <jderobot/laser.h>
#include <jderobot/encoders.h>
#include <jderobot/ptencoders.h>
#include <jderobot/pose3dmotors.h>
#include <jderobot/pose3dencoders.h>
#include <jderobot/pose3d.h>
#include <jderobot/pointcloud.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <string.h>
#include <buffer/RecorderInterface.h>
#include <ns/ns.h>
#include <pools/PoolsManager.h>
#include <pools/PoolWriteRGBD.h>
#include "recordergui.h"
#include "pools/PoolWriteImages.h"
#include "pools/poolWritePose3dEncoders.h"
#include "pools/poolWritePose3d.h"
#include "pools/poolWriteLasers.h"
#include "pools/poolWritePointCloud.h"
#include "pools/poolWriteEncoders.h"
#include "easyiceconfig/EasyIce.h" 

bool recording=false;
struct timeval inicio;
bool globalActive=true;
Ice::CommunicatorPtr ic;
bool killed=false;


recorder::PoolsManagerPtr manager;

void exitApplication(int s){
	globalActive=false;
	killed=true;

    manager->releaseAll();
   if (ic)
	   ic->destroy();
}


int main(int argc, char** argv){



	struct sigaction sigIntHandler;

   sigIntHandler.sa_handler = exitApplication;
   sigemptyset(&sigIntHandler.sa_mask);
   sigIntHandler.sa_flags = 0;

   sigaction(SIGINT, &sigIntHandler, NULL);

	struct timeval a, b;
	int cycle = 100;
	long totalb,totala;
	long diff;

    jderobot::ns* namingService = NULL;


    //INTERFACE DATA
	jderobot::EncodersDataPtr ed;
	jderobot::LaserDataPtr ld;
	jderobot::ImageDataPtr imageData;

	//pools

	pthread_attr_t attr;
	//images
	int nConsumers;
	size_t poolSize;
	int Hz = 10;
    //numero de lasers
	int pngCompressRatio;
	int jpgQuality;
    std::string fileFormat;
	std::vector<int> compression_params;
    
   
	//---------------- INPUT ARGUMENTS ---------------//

	if (argc<2){
		std::cout << std::endl << "USE: ./mycomponent --Ice.Config=mycomponent.cfg" << std::endl;
	}

	//---------------- INPUT ARGUMENTS -----------------//

	try{
        jderobot::Logger::initialize(argv[0]);
		//creamos el directorio principal
		Ice::PropertiesPtr prop;

		ic = EasyIce::initialize(argc,argv);

		prop = ic->getProperties();
        Ice::ObjectAdapterPtr adapter;


        Hz = prop->getPropertyAsInt("Recorder.Hz");
		cycle = 1000.0/Hz;
      
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
      

		int nCameras = prop->getPropertyAsIntWithDefault("Recorder.nCameras",0);
        nConsumers=prop->getPropertyAsIntWithDefault("Recorder.nConsumers",2);
        poolSize=prop->getPropertyAsIntWithDefault("Recorder.poolSize",10);
		fileFormat=prop->getProperty("Recorder.FileFormat");
		if (fileFormat.compare(std::string("png"))==0){
			pngCompressRatio=prop->getPropertyAsIntWithDefault("Recorder.PngCompression",3);
			compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(pngCompressRatio);
		}
		else if (fileFormat.compare(std::string("jpg"))==0){
			jpgQuality=prop->getPropertyAsIntWithDefault("Recorder.JpgQuality",95);
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(jpgQuality);
		}
		else{
			throw "File format is not valid";
		}

        std::string baseLogPath="./data/";

        manager= recorder::PoolsManagerPtr( new recorder::PoolsManager(attr,nConsumers));


        int bufferEnabled = prop->getPropertyAsIntWithDefault("Recorder.Buffer.Enabled",0);
        int bufferSeconds = prop->getPropertyAsIntWithDefault("Recorder.Buffer.Seconds",0);
        std::string videoMode =  prop->getProperty("Recorder.Buffer.Mode");


		for (int i=0; i< nCameras; i++){
            std::stringstream sProxy;
			// Get driver camera
			sProxy << "Recorder.Camera" << i+1 << ".Proxy";
			Ice::ObjectPrx camara = ic->propertyToProxy(sProxy.str());
			if (0==camara)
				throw "Could not create proxy to camera1 server";
			// cast to CameraPrx
			jderobot::CameraPrx cprxAux = jderobot::CameraPrx::checkedCast(camara);
			if (0== cprxAux)
				throw "Invalid proxy";

            std::stringstream sFormat;
            std::string imageFormat;
            sFormat << "Recorder.Camera" << i+1 << ".Format";
            imageFormat = prop->getProperty(sFormat.str());
			recorder::poolWriteImagesPtr temp = recorder::poolWriteImagesPtr( new recorder::poolWriteImages(cprxAux, Hz,poolSize,i+1,
                        imageFormat ,fileFormat ,compression_params,baseLogPath,(bufferEnabled == 0)? recorder::WRITE_FRAME : recorder::SAVE_BUFFER, bufferSeconds, videoMode));
            manager->addPool(recorder::IMAGES,temp);
		}





        if (bufferEnabled)
        {
            // Analyze EndPoint
            std::string Endpoints = prop->getProperty("Recorder.Endpoints");
            adapter =ic->createObjectAdapterWithEndpoints("Recorder", Endpoints);
            std::string name = prop->getProperty("Recorder.Name");
            LOG(INFO) << "Creating Recorder: " + name;
            recorder::RecorderInterface* recorder_prx = new recorder::RecorderInterface(manager);



            adapter->add(recorder_prx, ic->stringToIdentity(name));

            adapter->activate();


            // Naming Service
            int nsActive = prop->getPropertyAsIntWithDefault("NamingService.Enabled", 0);

            if (nsActive) {
                std::string ns_proxy = prop->getProperty("NamingService.Proxy");
                try {
                    namingService = new jderobot::ns(ic, ns_proxy);
                }
                catch (Ice::ConnectionRefusedException &ex) {
                    LOG(ERROR) << "Impossible to connect with NameService!";
                    exit(-1);
                }
                namingService->bind(name, Endpoints, recorder_prx->ice_staticId());
            }
        }



		int nLasers= prop->getPropertyAsInt("Recorder.nLasers");
		for (int i=0; i< nLasers; i++){
			std::stringstream sProxy;
			sProxy << "Recorder.Laser" << i+1 << ".Proxy";

			Ice::ObjectPrx baseLaser = ic->propertyToProxy(sProxy.str());
			if (0==baseLaser)
				throw "Could not create proxy with laser";

			recorder::poolWriteLasersPtr temp = recorder::poolWriteLasersPtr(new recorder::poolWriteLasers(baseLaser, Hz,poolSize,i+1,baseLogPath));
            manager->addPool(recorder::LASERS,temp);

		}


		int nPose3dEncoders= prop->getPropertyAsInt("Recorder.nPose3dEncoders");
		for (int i=0; i< nPose3dEncoders; i++){
			std::stringstream sProxy;
			sProxy << "Recorder.Pose3DEncoders" << i+1 << ".Proxy";

			Ice::ObjectPrx base = ic->propertyToProxy(sProxy.str());
			if (0==base)
				throw "Could not create proxy with pose3dencoders";
			recorder::poolWritePose3dEncodersPtr temp = recorder::poolWritePose3dEncodersPtr(new recorder::poolWritePose3dEncoders(base, Hz,poolSize,i+1,baseLogPath));
			manager->addPool(recorder::POSE3DENCODERS,temp);

		}

		int nPose3d= prop->getPropertyAsInt("Recorder.nPose3d");
		for (int i=0; i< nPose3d; i++){
			std::stringstream sProxy;
			sProxy << "Recorder.Pose3D" << i+1 << ".Proxy";

			Ice::ObjectPrx base = ic->propertyToProxy(sProxy.str());
			if (0==base)
				throw "Could not create proxy with pose3d";

			recorder::poolWritePose3dPtr temp = recorder::poolWritePose3dPtr(new recorder::poolWritePose3d(base, Hz,poolSize,i+1,baseLogPath));
			manager->addPool(recorder::POSE3D,temp);


		}

		int nEncoders= prop->getPropertyAsInt("Recorder.nEncoders");
		for (int i=0; i< nEncoders; i++){
			// Contact to ENCODERS interface
			std::stringstream sProxy;
			sProxy << "Recorder.Encoders" << i+1 << ".Proxy";

			Ice::ObjectPrx base = ic->propertyToProxy(sProxy.str());
			if (0==base)
				throw "Could not create proxy with encoders";

			recorder::poolWriteEncodersPtr temp = recorder::poolWriteEncodersPtr(new recorder::poolWriteEncoders(base, Hz,poolSize,i+1, baseLogPath));
            manager->addPool(recorder::ENCODERS,temp);

		}

		int nDepthSensors = prop->getPropertyAsIntWithDefault("Recorder.nDethSensors",0);
		for (int i=0; i< nDepthSensors; i++){
			std::stringstream sProxy;
			// Get driver camera
			sProxy << "Recorder.DepthSensor" << i+1 << ".Proxy";

			Ice::ObjectPrx kinect = ic->propertyToProxy(sProxy.str());
			if (0==kinect){
				throw "Could not create proxy with Kinect1";
			}
			recorder::poolWritePointCloudPtr temp = recorder::poolWritePointCloudPtr(new recorder::poolWritePointCloud(kinect, Hz,poolSize,i+1,baseLogPath));
            manager->addPool(recorder::POINTCLOUD,temp);
		}


		int nRGBDSensors = prop->getPropertyAsIntWithDefault("Recorder.nRGBDSensors",0);
		for (int i=0; i< nRGBDSensors; i++){
			std::stringstream sProxy;
			// Get driver camera
			sProxy << "Recorder.RGBD" << i+1 << ".Proxy";

			Ice::ObjectPrx base = ic->propertyToProxy(sProxy.str());
			if (0==base){
				throw "Could not create proxy with RGBD";
			}

			std::stringstream sFormat;
			std::string imageFormat;
			sFormat << "Recorder.RGBD" << i+1 << ".Format";
			imageFormat = prop->getProperty(sFormat.str());

			recorder::PoolWriteRGBDPtr temp = recorder::PoolWriteRGBDPtr( new recorder::PoolWriteRGBD(base, Hz,poolSize,i+1,
					imageFormat ,fileFormat ,compression_params,baseLogPath,(bufferEnabled == 0)? recorder::WRITE_FRAME : recorder::SAVE_BUFFER, bufferSeconds, videoMode));


			manager->addPool(recorder::RGBD,temp);
		}

        manager->createThreads();


        //****************************** Processing the Control ******************************///


		//---------------- ITERATIONS CONTROL -----------//
		//muestreo para el laser
		std::string robotName = prop->getPropertyWithDefault("Recorder.Hostname","localhost");
		std::string robotPort = prop->getPropertyWithDefault("Recorder.Port","9999");
      
		long timeRelative = 0;


		int guiActive=prop->getPropertyAsIntWithDefault("Recorder.GUI",0);
		recorder::recordergui *gui;

		if (guiActive){
	      gui = new recorder::recordergui();
		}
		long long int iteration=0;
		
		while(globalActive){
			//gui activado
			if (guiActive){
				gui->set_iteration(iteration);
				gui->update();
				globalActive=gui->get_active();
				recording=gui->get_recording();
			}
			else{
				recording=true;
			}

			if (recording){
				if (iteration==0){
					gettimeofday(&inicio,NULL);
                    //init the pools
                    manager->startRecording(inicio);
				}
				iteration++;
				gettimeofday(&b,NULL);
				totalb=b.tv_sec*1000000+b.tv_usec;
				//calculamos la velocidad de grabación actual
				float myfps=1000000./((float)totalb-(float)totala);
				if (guiActive){
					gui->set_fps((int)myfps);
				}
				gettimeofday(&a,NULL);
				totala=a.tv_sec*1000000+a.tv_usec;

				gettimeofday(&b,NULL);
				totalb=b.tv_sec*1000000+b.tv_usec;
				std::cout << "Recorder takes " << (totalb-totala)/1000 << " ms" << std::endl;

				diff = (totalb-totala)/1000;
				if(diff < 0 || diff > cycle)
					diff = cycle;
				else
					diff = cycle-diff;

				//Sleep Algorithm
				usleep(diff*1000);
				if(diff < 10)
					usleep(10*1000);
                timeRelative+= diff + (totalb-totala)/1000;
			}
			else{
				usleep(10*1000);
			}

		}

	//--------------ITERATIONS CONTROL-------------//
	}
	catch (const Ice::Exception& ex) {
		LOG(ERROR) << ex ;
   }
	catch (const char* msg) {
		LOG(ERROR) << msg;
	}

	if (!killed)
		exitApplication(1);
   
   return 0;
}
