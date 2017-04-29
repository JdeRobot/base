//
// Created by frivas on 30/04/17.
//

#include "ReplayerPose3DEncoders.h"

namespace replayer {
    ReplayerPose3DEncoders::ReplayerPose3DEncoders(std::string &propertyPrefix, Ice::CommunicatorPtr ic,
                                                             replayer::SyncControllerPtr syncController,
                                                             long long int initStateIN) :
            prefix(propertyPrefix),
            encData(new jderobot::Pose3DEncodersData()),
            syncController(syncController) {
        this->prop = ic->getProperties();
        this->dataPath = prop->getProperty(propertyPrefix + "Dir");
        this->initState = initStateIN;
        v = new Pose3DEncodersSyncTask(this, this->dataPath);
        v->start();
    }

    jderobot::Pose3DEncodersDataPtr ReplayerPose3DEncoders::getPose3DEncodersData(const Ice::Current &) {
        this->dataMutex.lock();
        jderobot::Pose3DEncodersDataPtr localData(encData);
        this->dataMutex.unlock();
        return localData;
    };

}