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
using namespace gazebo::common;


QuadrotorControl::QuadrotorControl():
    my_state(QuadrotorState::Unknown)
{}

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
#if 1
    _control_loop_hector(_info);
#else
    _control_loop_novel(_info);
#endif
}


void
QuadrotorControl::takeoff(){
    if (my_state == QuadrotorState::Landed || my_state == QuadrotorState::Landing){
        my_state = QuadrotorState::TakingOff;
        std::cout << "QuadrotorState::TakingOff" << std::endl;
    }
}

void
QuadrotorControl::land(){
    if (my_state == QuadrotorState::Flying || my_state == QuadrotorState::TakingOff){
        my_state = QuadrotorState::Landing;
        std::cout << "QuadrotorState::Landing" << std::endl;
    }
}


void
QuadrotorControl::_control_loop_novel(const gazebo::common::UpdateInfo & _info){
    //// Forces and velocities are handled bt a physics engine
    /// This means that we must apply a counter-gravity force
    /// each cicle to model "fly" state.
    /// ToDo[enhancement]: apply battery status to counter-gravity,
    /// so when battery is drained, force tends to zero and
    /// quadrotor will fall.

    // getting gravity dynamically to allow on-demand changes.
    Vector3 gravity = base_link->GetWorld()->GetPhysicsEngine()->GetGravity();
    Vector3 counter_gravity = mass*-gravity;
    //base_link->AddForce(counter_gravity);

    Pose pose = base_link->GetWorldPose();


    Vector3 vel_model = base_link->GetRelativeLinearVel();
    Vector3 vel_world = base_link->GetWorldLinearVel(); //pose.rot.RotateVectorReverse(vel_model);
    Vector3 up_down_vel = Vector3(0,0,0.001);
    //std::cout<<vel_world<<std::endl;


    switch(my_state){
    case Unknown:
        if (pose.pos.z > /*model.height*/ 1)
            my_state = QuadrotorState::Flying;
        else
            my_state = QuadrotorState::Landed;
        std::cout << "\tboostrap quadrotor state as " << my_state <<std::endl;
    break;
    case Landing:
        vel_model = vel_model-up_down_vel;// = pose.rot.RotateVector(vel_world-up_down_vel);
        base_link->SetLinearVel(vel_model);
        if (pose.pos.z < 0.8){
            my_state = QuadrotorState::Landed;
            std::cout << "QuadrotorState::Landed" << std::endl;
        }
    break;
    case TakingOff:

        vel_model = vel_model+up_down_vel;// pose.rot.RotateVector(vel_world+up_down_vel);
        base_link->SetLinearVel(vel_model);
        if (pose.pos.z >= 3){
            my_state = QuadrotorState::Flying;
            std::cout << "QuadrotorState::Flying" << std::endl;
        }
    break;
    }

    if (my_state != QuadrotorState::Landed){
        base_link->AddForce(counter_gravity);
    }


#if 0 //Testing
    switch(my_state){
    case Landed: takeoff() break;
    case Flying: land() break;
    }
#endif
}

