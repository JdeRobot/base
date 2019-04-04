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



#include "formula1/formula1control.hh"

using namespace formula1;
using namespace ignition::math;
using namespace gazebo::physics;
using namespace gazebo::common;
using namespace gazebo::transport;

enum {
    RIGHT,
    LEFT
};

Formula1Control::Formula1Control(){

}

Formula1Control::~Formula1Control(){}

void
Formula1Control::Load(ModelPtr model, sdf::ElementPtr _sdf){

    this->model = model;
    this->node = NodePtr(new Node());
    this->node->Init(this->model->GetWorld()->Name());

    if (!_sdf->HasElement("drive_right_joint"))
        gzerr << "motors plugin missing <drive_right_joint> element\n";
    if (!_sdf->HasElement("drive_left_joint"))
        gzerr << "motors plugin missing <drive_left_joint> element\n";
    if (!_sdf->HasElement("steering_right_joint"))
        gzerr << "motors plugin missing <steering_right_joint> element\n";
    if (!_sdf->HasElement("steering_left_joint"))
        gzerr << "motors plugin missing <steering_left_joint> element\n";

    std::cout<< _sdf->GetElement("drive_left_joint")->Get<std::string>()<< std::endl;

    this->driveLeftJoint = model->GetJoint(
        _sdf->GetElement("drive_left_joint")->Get<std::string>());
    this->driveRightJoint = model->GetJoint(
        _sdf->GetElement("drive_right_joint")->Get<std::string>());
    this->steeringLeftJoint = model->GetJoint(
        _sdf->GetElement("steering_left_joint")->Get<std::string>());
    this->steeringRightJoint = model->GetJoint(
        _sdf->GetElement("steering_right_joint")->Get<std::string>());

    if (!this->driveLeftJoint)
        gzerr << "Unable to find drive left joint["
            << _sdf->GetElement("drive_left_joint")->Get<std::string>() << "]\n";
    if (!this->driveRightJoint)
        gzerr << "Unable to find drive right joint["
            << _sdf->GetElement("drive_right_joint")->Get<std::string>() << "]\n";
    if (!this->steeringLeftJoint)
        gzerr << "Unable to find steering left joint["
            << _sdf->GetElement("steering_left_joint")->Get<std::string>() << "]\n";
    if (!this->steeringRightJoint)
        gzerr << "Unable to find steering right joint["
            << _sdf->GetElement("steering_right_joint")->Get<std::string>() << "]\n";
}

void
Formula1Control::Init(Formula1Sensors *sensors){
    this->sensors = sensors;


    this->wheelSeparation = this->driveLeftJoint->Anchor(0).Distance(this->driveRightJoint->Anchor(0));
    //std::cout << "Motors Separation:" << this->frontmotorseparation << std::endl;
    EntityPtr parent = boost::dynamic_pointer_cast<Entity > (this->driveLeftJoint->GetChild());

    Box bb = parent->BoundingBox();

    this->wheelRadius = bb.Size().Max() * 0.5;
    std::cout << "motors Diameter:" << this->wheelRadius * 2 << std::endl;

    robotMotors.v = 0;
    robotMotors.w = 0;
    robotMotors.wheelMin= steeringLeftJoint->LowerLimit(0);
    robotMotors.wheelMax= steeringLeftJoint->UpperLimit(0);
    robotMotors.targetRightSteerPos=robotMotors.targetLeftSteerPos=0;

    this->steeringRightJoint->SetUpperLimit(0,robotMotors.wheelMax);
    this->steeringRightJoint->SetLowerLimit(0,robotMotors.wheelMin);
    this->steeringLeftJoint->SetUpperLimit(0,robotMotors.wheelMax);
    this->steeringLeftJoint->SetLowerLimit(0,robotMotors.wheelMin);

    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Formula1Control::OnUpdate, this, _1));
}

void
Formula1Control::OnUpdate(const gazebo::common::UpdateInfo & /*_info*/){

            pthread_mutex_lock(&mutex);

            robotMotors.targetRightSteerPos=robotMotors.w*0.145-this->steeringRightJoint->Position(0);
            robotMotors.targetLeftSteerPos=robotMotors.w*0.145-this->steeringLeftJoint->Position(0);

            this->motorspeed = robotMotors.v*10;

            //DEPRECATED
            //this->frontLeftJoint->SetMaxForce(0,torque);
            //this->frontRightJoint->SetMaxForce(0,torque);
            this->driveLeftJoint->SetVelocity(0, motorspeed);
            this->driveRightJoint->SetVelocity(0, motorspeed);

            if(this->steeringRightJoint->Position(0) >= robotMotors.wheelMax
                    || this->steeringLeftJoint->Position(0) >= robotMotors.wheelMax
                    || this->steeringRightJoint->Position(0) <= robotMotors.wheelMin
                    || this->steeringLeftJoint->Position(0) <= robotMotors.wheelMin)
            {
                this->steeringLeftJoint->SetForce(0,0.00000);
                this->steeringRightJoint->SetForce(0,0.00000);

            }else
            {
                this->steeringLeftJoint->SetForce(0,robotMotors.targetLeftSteerPos);
                this->steeringRightJoint->SetForce(0,robotMotors.targetRightSteerPos);
            }

            pthread_mutex_unlock(&mutex);
}

void
Formula1Control::teleport(ignition::math::Pose3d pose){
    this->model->SetWorldPose(pose);
}
