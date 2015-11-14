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

#ifndef QUADROTORPLUGIN_H
#define QUADROTORPLUGIN_H

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/Pose.hh>

#include "quadrotor/quadrotorsensors.hh"


namespace quadrotor{

class QuadrotorPlugin : public gazebo::ModelPlugin {
public:
    QuadrotorPlugin();

protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void Init();
    void OnUpdate(const gazebo::common::UpdateInfo & _info);
    void Reset();

private:
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr updateConnection;
    QuadRotorSensors sensors;

protected:
    gazebo::math::Pose pose;

};

}//NS

#endif // QUADROTORPLUGIN_H
