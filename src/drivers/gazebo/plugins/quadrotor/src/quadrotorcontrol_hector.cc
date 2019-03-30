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
    ignition::math::Pose3d pose = base_link->WorldPose();
    ignition::math::Vector3d euler = pose.Rot().Euler();
    ignition::math::Vector3d linear_velocity = base_link->WorldLinearVel();
    ignition::math::Vector3d angular_velocity = base_link->WorldAngularVel();
    ignition::math::Vector3d acceleration = base_link->WorldLinearAccel();
    ignition::math::Vector3d inertia = inertia = inertial->PrincipalMoments();

    double velocity_command_linear_z = velocity_command.linear[2];
#ifdef VELOCITY_BASED_TAKEOFF
    // Inject landing/takingoff
    if (my_state == QuadrotorState::Landing)
        velocity_command_linear_z = std::min(velocity_command.linear[2], -land_speed);
    if (my_state == QuadrotorState::TakingOff)
        velocity_command_linear_z = std::max(velocity_command.linear[2], +takeoff_speed);
#endif

    // Get gravity
    ignition::math::Vector3d gravity = base_link->GetWorld()->Gravity();
    ignition::math::Vector3d gravity_body = pose.Rot().RotateVector(gravity);
    double gravity_module = gravity_body.Length();
    double load_factor = gravity_module * gravity_module / gravity.Dot(gravity_body);  // Get gravity

    // Rotate vectors to coordinate frames relevant for control
    ignition::math::Quaternion<double> heading_quaternion(cos(euler[2]/2),0,0,sin(euler[2]/2));
    ignition::math::Vector3d velocity_xy = heading_quaternion.RotateVectorReverse(linear_velocity);
    ignition::math::Vector3d acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
    ignition::math::Vector3d angular_velocity_body = pose.Rot().RotateVectorReverse(angular_velocity);

    // update controllers
    ignition::math::Vector3d force(ignition::math::Vector3d::Zero), torque(ignition::math::Vector3d::Zero);

    double pitch_command =  controllers.velocity_x.update(velocity_command.linear[0], velocity_xy[0], acceleration_xy[0], dt) / gravity_module;
    double roll_command  = -controllers.velocity_y.update(velocity_command.linear[1], velocity_xy[1], acceleration_xy[1], dt) / gravity_module;
    torque[0] = inertia[0] *  controllers.roll.update(roll_command, euler[0], angular_velocity_body[0], dt);
    torque[1] = inertia[1] *  controllers.pitch.update(pitch_command, euler[1], angular_velocity_body[1], dt);
    torque[2] = inertia[2] *  controllers.yaw.update(velocity_command.angular[2], angular_velocity[2], 0, dt);
    force[2]  = mass      * (controllers.velocity_z.update(velocity_command_linear_z,  linear_velocity[2], acceleration[2], dt) + load_factor * gravity_module);
    //if (max_force_ > 0.0 && force.z > max_force_) force.z = max_force_;
    if (force[2] < 0.0) force[2] = 0.0;

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

