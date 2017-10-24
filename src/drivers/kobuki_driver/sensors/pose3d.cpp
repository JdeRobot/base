#include "pose3d.h"

Pose3D::Pose3D(KobukiManager *kobuki)
{
    this->kobuki = kobuki;
}

int Pose3D::setPose3DData(const jderobot::Pose3DDataPtr&  pose3dData,
                             const Ice::Current&)
{
    return 0;
}


jderobot::Pose3DDataPtr Pose3D::getPose3DData(const Ice::Current&)
{

    mutex.lock();
    jderobot::Pose3DDataPtr pose3dData(new jderobot::Pose3DData());
    pose3dData->x = kobuki->getRobotX();
    pose3dData->y = kobuki->getRobotY();

    Eigen::Matrix<double, 3,1> m;
    m(0,0) = 0.0;
    m(1,0) = 0.0;
    m(2,0) = kobuki->getRobotTheta();

    jderobot::math::Quaternion<double> q = jderobot::math::Quaternion<double>::FromEuler(m);

    pose3dData->q0 = q.w();
    pose3dData->q1 = q.x();
    pose3dData->q2 = q.y();
    pose3dData->q3 = q.z();

    mutex.unlock();
    return pose3dData;
}

