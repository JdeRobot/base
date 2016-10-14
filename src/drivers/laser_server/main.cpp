#include <iostream>
#include <stdio.h>
#include <Ice/Ice.h>
//#include <IceUtil/IceUtil.h>
#include "easyiceconfig/EasyIce.h"
#include <laseri.h>
//#include <jderobot/laser.h>

using namespace std;

int main(int argc, char *argv[])
{
    Ice::CommunicatorPtr ic;
    try {
     //-----------------ICE----------------//
        ic = EasyIce::initialize(argc, argv);

		Ice::PropertiesPtr prop = ic->getProperties();

        std::string Endpoints = prop->getProperty("Laser.Endpoints");
        std::string laserControl_string = "Laser";

		Ice::ObjectAdapterPtr adapter =
            ic->createObjectAdapterWithEndpoints(laserControl_string, Endpoints);
        Ice::ObjectPtr object = new laser::LaserI(prop);
        adapter->add(object, ic->stringToIdentity(laserControl_string));
        adapter->activate();
		std::cout << "laser_server start on " <<Endpoints<<std::endl;
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
/*
#include <iostream>
#include <stdio.h>
#include <Ice/Ice.h>
//#include <IceUtil/IceUtil.h>
#include "easyiceconfig/EasyIce.h"
#include <hokuyo/hokuyomanager.h>
#include <jderobot/laser.h>
#include <math.h>

using namespace std;

//hokuyo::LaserScan  scan_;
//hokuyo::Laser laser_;
//hokuyo::LaserConfig laser_config_;

int main(int argc, char *argv[])
{
    //cout << "Hello World!" << endl;
    //laser_.open("/dev/ttyACM0");
    //std::string product_name=laser_.getProductName();
    //printf("Conectado con %s", product_name.c_str());
    //laser_.close();
    Ice::CommunicatorPtr ic;
    std::string devicePortPre="/dev/ttyACM";

//y = x*pi/180

    try {
     //-----------------ICE----------------//
        ic = Ice::initialize(argc, argv);

		Ice::PropertiesPtr prop = ic->getProperties();

        std::string Endpoints = prop->getProperty("Laser.Endpoints");
        std::string laserControl_string = "Laser";

        std::string deviceId = prop->getProperty("Laser.DeviceId");
		double min = (double)prop->getPropertyAsInt("Laser.MinAng");
		double max = (double)prop->getPropertyAsInt("Laser.MaxAng");
		double min_ang = min*M_PI/180;
		double max_ang = max*M_PI/180;

        cout << Endpoints << endl;

cout << "min: " << min_ang << endl;

cout << "max: " << max_ang << endl;

       hokuyo::HokuyoManager* manager = new hokuyo::HokuyoManager(deviceId, min_ang, max_ang, 0, -1);
	   
	   jderobot::LaserData *data = manager->getLaserData();

	   cout << data->numLaser << endl;

    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        exit(-1);
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        exit(-1);
    }
    exit(0);
}*/
