/*
 *  Copyright (C) 1997-2015 JDE Developers Team
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
 *  Authors :
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */


#include "quadrotor/quadrotorice.hh"

using namespace quadrotor;
using namespace quadrotor::interfaces;
using namespace Ice;


QuadrotorIce::QuadrotorIce(CommunicatorPtr ic, const QuadRotorSensors *sensors, QuadrotorControl *control):
    ic(ic),
    sensor(sensors),
    control(control)
{
}

QuadrotorIce::~QuadrotorIce(){
    if (ice_thread!=0)
        delete ice_thread;
}

void
QuadrotorIce::start(){
    boost::thread *ice_thread = new boost::thread(boost::bind(&QuadrotorIce::run, this));
}

void
QuadrotorIce::run(){
    prop = ic->getProperties();

    adapter = ic->createObjectAdapter("Quadrotor.Adapter");
    std::cout << "Ice adapter listening at " << std::endl;
    std::cout << "\t" << adapter->getEndpoints()[0]->toString() << std::endl;

    std::string name;
    name = prop->getProperty("Quadrotor.Pose3D.Name");
    ObjectPtr posei = new Pose3DI(sensor);
    adapter->add(posei, ic->stringToIdentity(name));

    ObjectPtr navdatai = new NavdataI(sensor);
    name = prop->getProperty("Quadrotor.Navdata.Name");
    adapter->add(navdatai, ic->stringToIdentity(name));

    ObjectPtr dronecontroli = new DroneControlI(control);
    name = prop->getProperty("Quadrotor.Extra.Name");
    adapter->add(dronecontroli, ic->stringToIdentity(name));

    adapter->activate();

    std::cout<< "Ice booststrap done." << std::endl;

    ic->waitForShutdown();
}
