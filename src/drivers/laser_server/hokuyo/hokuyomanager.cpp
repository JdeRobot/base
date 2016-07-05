#include "hokuyomanager.h"

namespace hokuyo{
    HokuyoManager::HokuyoManager(std::string deviceId, double min_ang, double max_ang, int clustering, int timeout, int faceup)
	{
		std::string port = DEVICEPORTPRE + deviceId;
		
    	laser_.open(port.c_str());
		laser_.laserOn();
		laser_.setToSCIP2();
		this->min_ang = min_ang;
		this->max_ang = max_ang;
		this->clustering = clustering;
		this->timeout = timeout;
        this->faceup = faceup;
std::cout << "open: " << port << std::endl;
		hokuyo:LaserConfig config;
		laser_.getConfig(config);
std::cout<< "min_range: " << config.min_range << std::endl;
std::cout<< "max_range: " << config.max_range << std::endl;
std::cout<< "range_res: " << config.range_res << std::endl;
	}

	HokuyoManager::~HokuyoManager()
	{
		laser_.laserOff();	    
		laser_.close();
	}

	void
	HokuyoManager::laserScan2LaserData(hokuyo::LaserScan scan, jderobot::LaserData *data)
	{
	    data->numLaser = scan.ranges.size();
	    data->distanceData.resize(sizeof(int)*data->numLaser);
        if (this->faceup!=0){
            for(int i = 0 ; i < data->numLaser; i++){
                if (std::numeric_limits<float>::infinity() == scan.ranges[i]){
                    data->distanceData[i] = scan.config.max_range*1000;
                }else if (scan.ranges[i] < 0){
                    data->distanceData[i] = 0;
                }else{
                    data->distanceData[i] = scan.ranges[i]*1000;
                }
            }
        }else{
            for(int i = 0 ; i < data->numLaser; i++){
                if (std::numeric_limits<float>::infinity() == scan.ranges[i]){
                    data->distanceData[data->numLaser-1-i] = scan.config.max_range*1000;
                }else if (scan.ranges[i] <0){
                    data->distanceData[data->numLaser-1-i] = 0;
                }else{
                    data->distanceData[data->numLaser-1-i] = scan.ranges[i]*1000;
                }
            }
       }
	}

	jderobot::LaserData
	*HokuyoManager::getLaserData()
	{
	    jderobot::LaserData *data = new jderobot::LaserData();
	    hokuyo::LaserScan  scan;
	    int res;
	    mutex.lock();
	    res = laser_.pollScan(scan,this->min_ang,this->max_ang,this->clustering,this->timeout);
	
	    mutex.unlock();

	    laserScan2LaserData(scan, data);
	    return data;
	}
}

