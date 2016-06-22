#ifndef POSE3D_H
#define POSE3D_H

//boost
#include <boost/signals2/mutex.hpp>

//Pose3D
#include <jderobot/pose3d.h>

//ICE
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include "../kobukimanager.h"
#include "quaternion.h"

class Pose3D: virtual public jderobot::Pose3D
{
public:
    Pose3D(KobukiManager* kobuki);

    virtual jderobot::Pose3DDataPtr getPose3DData(const Ice::Current&);

    virtual int setPose3DData(const jderobot::Pose3DDataPtr&  pose3dData,
                                 const Ice::Current&);

    private: boost::signals2::mutex mutex; ///< Mutex for thread-safe access to internal data.


    private: KobukiManager* kobuki;

};

#endif // POSE3D_H
