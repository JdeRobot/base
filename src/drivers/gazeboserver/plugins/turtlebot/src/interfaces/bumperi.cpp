#include "turtlebot/interfaces/bumperi.h"

using namespace turtlebot::interfaces;
using namespace jderobot;


BumperI::BumperI (const TurtlebotSensors *sensor):
    bumperData(new BumperData()),
    sensor(sensor)
{}

BumperI::~BumperI ()
{}

BumperDataPtr
BumperI::getBumperData ( const Ice::Current& ){

    bumperdata->numContacts = sensor->bumperData.numContacts;
    bumperdata->contact1 = sensor->bumperdata.contact1;
    bumperdata->contact2 = sensor->bumperdata.contact2;

    return bumperdata;
}







