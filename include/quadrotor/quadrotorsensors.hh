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


#ifndef QUADROTORSENSORS_H
#define QUADROTORSENSORS_H


#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/sensors.hh>


namespace quadrotor{

class QuadRotorSensors
{
public:
    QuadRotorSensors();

    void Load(gazebo::physics::ModelPtr model);

    void debugInfo();

public:
    gazebo::sensors::CameraSensorPtr cam_ventral;
    gazebo::sensors::CameraSensorPtr cam_frontal;
    gazebo::sensors::SonarSensorPtr sonar;
    gazebo::sensors::ImuSensorPtr imu;

private:
    gazebo::physics::ModelPtr model;
    uint32_t base_link_id;

};

}//NS

#endif // QUADROTORSENSORS_H
