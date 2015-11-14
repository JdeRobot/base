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


#include "quadrotor/quadrotorsensors.hh"


using namespace quadrotor;
using namespace gazebo::physics;
using namespace gazebo::sensors;

QuadRotorSensors::QuadRotorSensors()
{

}

void
QuadRotorSensors::Load(ModelPtr model){
    this->model = model;
    this->base_link_id = model->GetChildLink("base_link")->GetId();

    SensorManager *sm = SensorManager::Instance();

    for (SensorPtr s: sm->GetSensors()){
        if (s->GetParentId() != base_link_id) continue;
        std::string name = s->GetName();
        if (name.find("imu_sensor") != -1)
            imu = boost::static_pointer_cast<ImuSensor>(s);
        if (name.find("sonar") != -1)
            sonar = boost::static_pointer_cast<SonarSensor>(s);
        if (name.find("frontal") != -1)
            cam_frontal = boost::static_pointer_cast<CameraSensor>(s);
        if (name.find("ventral") != -1)
            cam_ventral = boost::static_pointer_cast<CameraSensor>(s);
    }
}


void
QuadRotorSensors::Init(){
    sub_cam_frontal = cam_frontal->ConnectUpdated(
        boost::bind(&QuadRotorSensors::_on_cam_frontal, this));

    sub_cam_ventral = cam_frontal->ConnectUpdated(
        boost::bind(&QuadRotorSensors::_on_cam_ventral, this));

    sub_sonar = cam_frontal->ConnectUpdated(
        boost::bind(&QuadRotorSensors::_on_sonar, this));

    sub_imu = cam_frontal->ConnectUpdated(
        boost::bind(&QuadRotorSensors::_on_imu, this));
}

void
QuadRotorSensors::debugInfo(){
    std::cout << "Sensors of " << model->GetName() << std::endl;
    boost::format fmt("\t%1% (id: %2%)\n");

    std::cout << fmt % cam_ventral->GetName() % cam_ventral->GetId();
    std::cout << fmt % cam_frontal->GetName() % cam_frontal->GetId();
    std::cout << fmt % sonar->GetName() % sonar->GetId();
    std::cout << fmt % imu->GetName() % imu->GetId();
}


void
QuadRotorSensors::_on_cam_frontal(){
    //// Assumption: Camera (raw) data is constant, so after Camera boostrap
    /// Camera.data will be a constant pointer.
    /// Therefore, we can use the cv::Mat constructor to simply wrap this
    /// data and avoid copy overload.
    /// Warning: thread safe is not supplied, but one can never ensure it
    /// if CameraSensor thread is overriding data and there are no option
    /// to intercept this update.
    /// Warning 2: char* is NULL until CameraSensor.OnUpdate(), so it is not
    /// possible to bootstrap at Load neither Init step.

    if (img_frontal.empty()){
        const unsigned char *data = cam_frontal->GetImageData();
        if (data == 0)
            return;

        std::cout<<"\tbootstrap cam_frontal"<<std::endl;
        uint32_t h = cam_frontal->GetImageHeight();
        uint32_t w = cam_frontal->GetImageWidth();
        img_frontal = cv::Mat(h, w, CV_8UC3, (uint8_t*)data);

        sub_cam_frontal->~Connection(); // close connection
    }
}

void
QuadRotorSensors::_on_cam_ventral(){
    if (img_ventral.empty()){
        const unsigned char *data = cam_ventral->GetImageData();
        if (data == 0)
            return;

        std::cout<<"\tbootstrap cam_ventral"<<std::endl;
        uint32_t h = cam_ventral->GetImageHeight();
        uint32_t w = cam_frontal->GetImageWidth();
        img_ventral = cv::Mat(h, w, CV_8UC3, (uint8_t*)data);

        sub_cam_ventral->~Connection();
    }
}


void
QuadRotorSensors::_on_sonar(){
    //ToDO:
    altitude = sonar->GetRangeMin();
}

void
QuadRotorSensors::_on_imu(){
    pose = model->GetWorldPose();
    pose.rot = imu->GetOrientation();
}
