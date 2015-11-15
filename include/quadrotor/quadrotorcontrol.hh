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

#ifndef QUADROTORCONTROL_H
#define QUADROTORCONTROL_H


#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>


namespace quadrotor{

class QuadrotorControl {
public:
    QuadrotorControl();
    virtual ~QuadrotorControl();


    void takeoff();
    void land();

private:
    double mass;


/// Gazebo
public:
    void Load(gazebo::physics::LinkPtr _base_link, sdf::ElementPtr _sdf);
    void Init();
    void OnUpdate(const gazebo::common::UpdateInfo & _info);


private:
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::physics::LinkPtr base_link;
    gazebo::physics::InertialPtr inertial;

};


typedef boost::shared_ptr<QuadrotorControl> QuadrotorControlPtr;

}//NS

#endif // QUADROTORPLUGIN_H
