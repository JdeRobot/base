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



#include "quadrotor/quadrotorcontrol.hh"


using namespace quadrotor;
using namespace gazebo::math;
using namespace gazebo::physics;


QuadrotorControl::QuadrotorControl(){
}

QuadrotorControl::~QuadrotorControl(){

}

void
QuadrotorControl::Load(LinkPtr _base_link, sdf::ElementPtr _sdf){
    base_link = _base_link;
    inertial = _base_link->GetInertial();
    mass = inertial->GetMass();
}

void
QuadrotorControl::Init(){
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&QuadrotorControl::OnUpdate, this, _1));
}

void
QuadrotorControl::OnUpdate(const gazebo::common::UpdateInfo & _info){
    //// Forces and velocities are handled bt a physics engine
    /// This means that we must apply a counter-gravity force
    /// each cicle to model "fly" state.
    /// ToDo[enhancement]: apply battery status to counter-gravity,
    /// so when battery is drained, force tends to zero and
    /// quadrotor will fall.

    // getting gravity dynamically to allow on-demand changes.
    Vector3 gravity = base_link->GetWorld()->GetPhysicsEngine()->GetGravity();
    Vector3 counter_gravity = mass*-gravity;

    Pose pose = base_link->GetWorldPose();

    base_link->AddForce(counter_gravity);
}


void
QuadrotorControl::takeoff(){

}

void
QuadrotorControl::land(){

}
