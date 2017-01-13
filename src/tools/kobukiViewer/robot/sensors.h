#ifndef SENSORS_H
#define SENSORS_H

#include <QMutex>

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <jderobot/pose3d.h>

#include <jderobot/types/laserData.h>
#include <jderobot/types/image.h>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <jderobot/comm/laserClient.hpp>
#include <jderobot/comm/cameraClient.hpp>

class Sensors
{
public:
    Sensors(Ice::CommunicatorPtr ic);

    void update();

    float getRobotPoseX();
    float getRobotPoseY();
    float getRobotPoseTheta();
    JdeRobotTypes::LaserData getLaserData();

    JdeRobotTypes::Image getImage1();
    JdeRobotTypes::Image getImage2();


private:

    QMutex mutex;

    Ice::CommunicatorPtr ic;

    // ICE INTERFACES
    jderobot::Pose3DPrx p3dprx;
    jderobot::Pose3DDataPtr pose3ddata;

    //ICE interfaces available for connection on demand
    bool pose3dON;

    //ROBOT POSE
    double robotx;
    double roboty;
    double robottheta;

    //LASER DATA
    JdeRobotComm::LaserClient* laserClient;

    JdeRobotComm::CameraClient* camera1;
    JdeRobotComm::CameraClient* camera2;


};

#endif // SENSORS_H
