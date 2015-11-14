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



#include "quadrotor/quadrotorplugin.hh"

GZ_REGISTER_MODEL_PLUGIN(quadrotor::QuadrotorPlugin)

using namespace quadrotor;
using namespace gazebo::physics;
using namespace gazebo::math;
using namespace gazebo::event;
using namespace gazebo::common;


QuadrotorPlugin::QuadrotorPlugin()
{

}


void
QuadrotorPlugin::Load(ModelPtr _model, sdf::ElementPtr _sdf){
    model = _model;
    sensors.Load(model);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = Events::ConnectWorldUpdateBegin(
        boost::bind(&QuadrotorPlugin::OnUpdate, this, _1));
}


void
QuadrotorPlugin::Init(){
std::cout << "QuadrotorPlugin::Init()" << std::endl;
    sensors.debugInfo();
    sensors.Init();

    sensors.cam_frontal->SetActive(true);
    sensors.cam_ventral->SetActive(true);
}


void
QuadrotorPlugin::OnUpdate(const UpdateInfo & _info){
//    std::cout << "QuadrotorPlugin::OnUpdate()" << std::endl;
//    std::cout << "\t" << _info.simTime << std::endl;
}


void
QuadrotorPlugin::Reset(){

}
