#include "turtlebot/interfaces/laseri.h"

using namespace turtlebot::interfaces;
using namespace jderobot;


LaserI::LaserI (const TurtlebotSensors *sensor):
    laserdata(new LaserData()),
    sensor(sensor)
{}

LaserI::~LaserI ()
{}

LaserDataPtr
LaserI::getLaserData ( const Ice::Current& ){

    laserdata->numLaser = sensor->laserValues.size();
    laserdata->distanceData.resize(sizeof(int)*laserdata->numLaser);

    //Update laser values
    for(int i = 0 ; i < laserdata->numLaser; i++){
       laserdata->distanceData[i] = sensor->laserValues[i]*1000;
    }
    return laserdata;
}
