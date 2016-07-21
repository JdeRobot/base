#include "laseri.h"

namespace laser
{

	LaserI::LaserI(Ice::PropertiesPtr prop)
	{
		std::string model = prop->getProperty("Laser.Model");
		
		if ("Hokuyo"==model || "hokuyo"==model){
	        std::string deviceId = prop->getProperty("Laser.DeviceId");
			double min = (double)prop->getPropertyAsInt("Laser.MinAng");
			double max = (double)prop->getPropertyAsInt("Laser.MaxAng");
			int clustering = prop->getPropertyAsInt("Laser.Clustering");
            int faceup = prop->getPropertyAsInt("Laser.FaceUp");
			double min_ang = min*M_PI/180;
			double max_ang = max*M_PI/180;
            this->manager = new hokuyo::HokuyoManager(deviceId, min_ang, max_ang, clustering, -1, faceup);
		}else{
			throw std::invalid_argument( model + " laser is not allowed" );
		}
	}
	
	LaserI::~LaserI()
	{
	}

	jderobot::LaserDataPtr LaserI::getLaserData(const Ice::Current&)
	{
		    jderobot::LaserDataPtr laserData (manager->getLaserData()); 
			return laserData;
	}
}
