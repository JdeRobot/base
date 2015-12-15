/*
 *  Copyright (C) 2015 JDE Developers Team
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

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>

// Ice
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <easyiceconfig/EasyIce.h>
#include <easyiceconfig/debug.hpp>

// Jderobot
#include <jderobot/pose3d.h>



namespace kinect{

class KinectPlugin :
        public gazebo::ModelPlugin,
        public jderobot::Pose3D
{
public:
    KinectPlugin(){
        ice_pose3ddata = new jderobot::Pose3DData(0,0,0,0,0,0,0,0);
    }

    ~KinectPlugin(){}

    std::string _log_prefix;

private:


/// Gazebo
protected:
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf){
        _log_prefix = "["+_model->GetName()+"] ";
        model = _model;

        InitializeIce(_sdf);

        sigintConnection = gazebo::event::Events::ConnectSigInt(
            boost::bind(&KinectPlugin::OnSigInt, this));
    }

    void Init(){
        updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&KinectPlugin::OnUpdate, this, _1));
    }

    void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/){
        gazebo::math::Pose pose = model->GetWorldPose();
        ice_pose3ddata = new jderobot::Pose3DData(pose.pos.x, pose.pos.y, pose.pos.z, 1, pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z);
    }

    void Reset(){}


    void OnSigInt(){
        if (ic){
            ic->shutdown();
            std::cout<<_log_prefix << "Ice is down now" << std::endl;
        }
    }

/// Ice
    void InitializeIce(sdf::ElementPtr _sdf){
        std::cout << _log_prefix << "KinectPlugin::InitializeIce()" << std::endl;
        std::string iceConfigFile = "flyingKinect2.cfg";
        if(_sdf->HasElement("iceConfigFile"))
            iceConfigFile =  _sdf->GetElement("iceConfigFile")->GetValue()->GetAsString();
        std::cout << _log_prefix << "\tconfig: "<< iceConfigFile << std::endl;

        Ice::InitializationData id;
        id.properties = Ice::createProperties();
        easyiceconfig::loader::loadIceConfig(iceConfigFile, id.properties);
        ic = Ice::initialize(id);

        Ice::ObjectAdapterPtr adapter;
        try{
        adapter = ic->createObjectAdapter("Kinect");
        adapter->add(this, ic->stringToIdentity("Pose3d"));
        adapter->activate();
        }catch(Ice::Exception &ex){ std::cout<< ex << std::endl;}

        std::cout<<_log_prefix << "Ice adapter listening at " << std::endl;
        std::cout<<_log_prefix << "\t" << adapter->getEndpoints()[0]->toString() << std::endl;
    }


/// Ice (Pose3D)
private:
    jderobot::Pose3DDataPtr ice_pose3ddata;
public:
    jderobot::Pose3DDataPtr
    getPose3DData ( const Ice::Current& ){
        return ice_pose3ddata;
    }

    Ice::Int
    setPose3DData ( const jderobot::Pose3DDataPtr & pose3dData,
                                     const Ice::Current& ){
        gazebo::math::Pose pose(
                    gazebo::math::Vector3(pose3dData->x, pose3dData->y, pose3dData->z),
                    gazebo::math::Quaternion(pose3dData->q0, pose3dData->q1, pose3dData->q2, pose3dData->q3)
        );

        model->SetWorldPose(pose);
        return 0;
    }

private:
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr updateConnection;
    gazebo::event::ConnectionPtr sigintConnection;


/// Ice
protected:
    Ice::CommunicatorPtr ic;

private:

};

GZ_REGISTER_MODEL_PLUGIN(kinect::KinectPlugin)

}//NS
