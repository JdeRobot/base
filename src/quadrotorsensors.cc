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


#include "quadrotor/quadrotorsensors.hh"


#include <boost/format.hpp>
#include <gazebo/sensors/SensorManager.hh>
#include <iostream>


using namespace quadrotor;
using namespace gazebo::physics;
using namespace gazebo::sensors;

QuadRotorSensors::QuadRotorSensors()
{

}

void
QuadRotorSensors::Load(ModelPtr model){
    this->model = model;
    this->base_link_id = model->GetChildLink("base_link")->GetId();

    SensorManager *sm = SensorManager::Instance();

    for (SensorPtr s: sm->GetSensors()){
        if (s->GetParentId() != base_link_id) continue;
        std::string name = s->GetName();
        if (name.find("imu_sensor") != -1)
            imu = boost::static_pointer_cast<ImuSensor>(s);
        if (name.find("sonar") != -1)
            sonar = boost::static_pointer_cast<SonarSensor>(s);
        if (name.find("frontal") != -1)
            cam_frontal = boost::static_pointer_cast<CameraSensor>(s);
        if (name.find("ventral") != -1)
            cam_ventral = boost::static_pointer_cast<CameraSensor>(s);
    }
}

void
QuadRotorSensors::debugInfo(){
    std::cout << "Sensors of " << model->GetName() << std::endl;
    boost::format fmt("\t%1% (id: %2%)\n");

    std::cout << fmt % cam_ventral->GetName() % cam_ventral->GetId();
    std::cout << fmt % cam_frontal->GetName() % cam_frontal->GetId();
    std::cout << fmt % sonar->GetName() % sonar->GetId();
    std::cout << fmt % imu->GetName() % imu->GetId();
}
