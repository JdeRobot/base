#include <iostream>
#include <stdio.h>
#include <Ice/Ice.h>
#include "easyiceconfig/EasyIce.h"
#include <evicam_driver/interfaces/pantilti.hpp>
#include <EVI-D100P.h>

using namespace std;

EVI_D100P* initCam (std::string device){
    EVI_D100P* cam = new EVI_D100P();

    

    if(cam == NULL)
    {
      cerr<<"Error new cam\n";
      exit(-1);
    }

    if(cam->Init() != 1)
    {
      cerr<<"Error cam Init\n";
      exit(-1);
    }

    char *devicec = new char[device.length() + 1];
    strcpy(devicec, device.c_str());

    if(cam->Open(1, devicec) != 1)
    {
      cerr<<"Error cam Open\n";
      exit(-1);
    }

    delete [] devicec;

    int n = cam->PowerInq();
    if(n == -1){
        cerr<<"Error cam Looking Power\n";
        exit(-1);
    }
    else{
        if(n == EVILIB_OFF)
        {
            cam->Power(EVILIB_ON);
        }
    }
    cam->Home();
    cam->Focus(EVILIB_AUTO, 0, EVILIB_WAIT_COMP);

    std::cout << "+++ MAX/MIN Values +++" << std::endl;
    std::cout << "+ Min Pan: " << EVILIB_minpan << "º     +" << std::endl;
    std::cout << "+ Max Pan: " << EVILIB_maxpan << "º      +" << std::endl;
    std::cout << "+ Min Pan speed: " << EVILIB_min_pspeed << "   +" << std::endl;
    std::cout << "+ Max Pan speed: " << EVILIB_max_pspeed << "  +" << std::endl;
    std::cout << "+ Min Tilt: " << EVILIB_mintilt << "º     +" << std::endl;
    std::cout << "+ Max Tilt: " << EVILIB_maxtilt << "º      +" << std::endl;
    std::cout << "+ Min Tilt speed: " << EVILIB_min_tspeed << "  +" << std::endl;
    std::cout << "+ Max Tilt speed: " << EVILIB_max_tspeed << " +" << std::endl;
    std::cout << "++++++++++++++++++++++" << std::endl;

    return cam;



}

int main(int argc, char *argv[])
{
    Ice::CommunicatorPtr ic;
    EVI_D100P *cam = NULL; // Allow access to the camera
    try {
     //-----------------ICE----------------//
        ic = EasyIce::initialize(argc, argv);

		Ice::PropertiesPtr prop = ic->getProperties();

        std::string Endpoints = prop->getProperty("PanTilt.Endpoints");
        std::string ptControl_string = "PanTilt";

        std::string device = prop->getProperty("PanTilt.Device");

        cam = initCam(device);

		Ice::ObjectAdapterPtr adapter =
            ic->createObjectAdapterWithEndpoints(ptControl_string, Endpoints);
        Ice::ObjectPtr object = new pantilt::PanTiltI(cam);
        adapter->add(object, ic->stringToIdentity(ptControl_string));
        adapter->activate();
		std::cout << "Evicam_driver start on " <<Endpoints<<std::endl;
        ic->waitForShutdown();
    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        exit(-1);
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        exit(-1);
    }

	if (ic) {
        try {
            ic->destroy();
        } catch (const Ice::Exception& e) {
            std::cerr << e << std::endl;
        }
    }


    exit(0);
}