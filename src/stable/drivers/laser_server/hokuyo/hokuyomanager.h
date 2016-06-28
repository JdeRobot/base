#ifndef HOKUYOMANAGER_H
#define HOKUYOMANAGER_H

#include <stdio.h>
#include <iostream>
#include "hokuyo.h"
#include "../lasermanager.h"
#include <jderobot/laser.h>

//boost
#include <boost/signals2/mutex.hpp>

namespace hokuyo{

	const std::string DEVICEPORTPRE="/dev/ttyACM";


	class HokuyoManager : public LaserManager	
	{
	public:
        HokuyoManager(std::string deviceId, double min_ang, double max_ang, int clustering, int timeout, int faceup);
    	~HokuyoManager();
    	jderobot::LaserData *getLaserData();
    	//void update();
	private:

    	void laserScan2LaserData(hokuyo::LaserScan scan, jderobot::LaserData *data);
    	hokuyo::Laser laser_;
    	boost::signals2::mutex mutex; ///< Mutex for thread-safe access to internal data.
    	double min_ang;
    	double max_ang;
    	int clustering;
    	int timeout;
        int faceup;
	};
}

#endif // HOKUYOMANAGER_H
