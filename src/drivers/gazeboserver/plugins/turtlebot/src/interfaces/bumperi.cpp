#include "turtlebot/interfaces/bumperi.h"

using namespace turtlebot::interfaces;
using namespace jderobot;


BumperI::BumperI (const TurtlebotSensors *sensor):
    bumperdata(new BumperData()),
    sensor(sensor)
{}

BumperI::~BumperI ()
{}

BumperDataPtr
BumperI::getBumperData ( const Ice::Current& ){

    bumperdata->bumper = sensor->bumperData.bumper;
    bumperdata->state = sensor->bumperData.state;

    return bumperdata;
}







