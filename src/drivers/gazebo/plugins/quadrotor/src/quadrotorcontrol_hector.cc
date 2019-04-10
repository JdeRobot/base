//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

/*
* quadrotor motion controller:
*
* This software is a fragment of original source
*
* Created on: Oct 22, 2012
* Author: Hongrong huang
*
* Adapted on: Nov 15, 2015
* Author: Victor Arribas
*
*/

#include "quadrotor/quadrotorcontrol.hh"

/// Constant velocity landing to avoid previous fall down behavior.
#define VELOCITY_BASED_TAKEOFF

using namespace quadrotor;
using namespace ignition::math;
using namespace gazebo::physics;
using namespace gazebo::common;


void
QuadrotorControl::_control_loop_hector(const gazebo::common::UpdateInfo & _info){
    double dt = (_info.simTime - last_simTime).Double();
    last_simTime = _info.simTime;
    if (dt == 0.0) return;

    // Max force (vel<=10m/s)
    //double max_force_ = mass*10/dt;

    // Get kinematics
    Pose3d pose = base_link->WorldPose();
    Vector3d euler = pose.Rot().Euler();
    Vector3d linear_velocity = base_link->WorldLinearVel();
    Vector3d angular_velocity = base_link->WorldAngularVel();
    Vector3d acceleration = base_link->WorldLinearAccel();
    Vector3d inertia = inertia = inertial->PrincipalMoments();

    double velocity_command_linear_z = velocity_command.linear.Z();
#ifdef VELOCITY_BASED_TAKEOFF
    // Inject landing/takingoff
    if (my_state == QuadrotorState::Landing)
        velocity_command_linear_z = std::min(velocity_command.linear.Z(), -land_speed);
    if (my_state == QuadrotorState::TakingOff)
        velocity_command_linear_z = std::max(velocity_command.linear.Z(), +takeoff_speed);
#endif

    // Get gravity
    Vector3d gravity = base_link->GetWorld()->Gravity();
    Vector3d gravity_body = pose.Rot().RotateVector(gravity);
    double gravity_module = gravity_body.Length();
    double load_factor = gravity_module * gravity_module / gravity.Dot(gravity_body);  // Get gravity

    // Rotate vectors to coordinate frames relevant for control
    Quaterniond heading_quaternion(cos(euler.Z()/2),0,0,sin(euler.Z()/2));
    Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(linear_velocity);
    Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

    // update controllers
    Vector3d force(Vector3d::Zero), torque(Vector3d::Zero);

    double pitch_command =  controllers.velocity_x.update(velocity_command.linear.X(), velocity_xy.X(), acceleration_xy.X(), dt) / gravity_module;
    double roll_command  = -controllers.velocity_y.update(velocity_command.linear.Y(), velocity_xy.Y(), acceleration_xy.Y(), dt) / gravity_module;
    torque.X() = inertia.X() *  controllers.roll.update(roll_command, euler.X(), angular_velocity_body.X(), dt);
    torque.Y() = inertia.Y() *  controllers.pitch.update(pitch_command, euler.Y(), angular_velocity_body.Y(), dt);
    torque.Z() = inertia.Z() *  controllers.yaw.update(velocity_command.angular.Z(), angular_velocity.Z(), 0, dt);
    force.Z()  = mass      * (controllers.velocity_z.update(velocity_command_linear_z,  linear_velocity.Z(), acceleration.Z(), dt) + load_factor * gravity_module);
    //if (max_force_ > 0.0 && force.Z() > max_force_) force.Z() = max_force_;
    if (force.Z() < 0.0) force.Z() = 0.0;

    if (my_state != QuadrotorState::Landed){
        base_link->AddRelativeForce(force);
        base_link->AddRelativeTorque(torque);
    }

#ifndef VELOCITY_BASED_TAKEOFF
    if (my_state == QuadrotorState::Landing){
        base_link->AddRelativeForce(-0.5*force);
        base_link->AddRelativeTorque(-0.5*torque);
    }

    if (my_state == QuadrotorState::TakingOff){
        base_link->AddRelativeForce(0.5*force);
        base_link->AddRelativeTorque(0.5*torque);
    }
#endif
}

