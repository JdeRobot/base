//
// Created by frivas on 30/04/17.
//

#include "ReplayerPose3D.h"


namespace  replayer {
    ReplayerPose3D::ReplayerPose3D(std::string &propertyPrefix, Ice::CommunicatorPtr ic,
                                             replayer::SyncControllerPtr syncController, long long int initStateIN) :
            prefix(propertyPrefix),
            pose3dData(new jderobot::Pose3DData()),
            syncController(syncController) {
        this->prop = ic->getProperties();
        this->dataPath = prop->getProperty(propertyPrefix + "Dir");
        this->initState = initStateIN;
        v = new Pose3DEncodersSyncTask(this, this->dataPath);
        v->start();
    }

    jderobot::Pose3DDataPtr ReplayerPose3D::getPose3DData(const Ice::Current &) {
        this->dataMutex.lock();
        jderobot::Pose3DDataPtr localData(pose3dData);
        this->dataMutex.unlock();
        return localData;
    };

}