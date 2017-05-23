//
// Created by frivas on 29/04/17.
//

#include "ReplayerPointCloud.h"



namespace replayer {
    replayer::ReplayerPointCloud::ReplayerPointCloud(std::string &propertyPrefix, Ice::CommunicatorPtr ic,
                                                     replayer::SyncControllerPtr syncController,
                                                     long long int initStateIN) :
            prefix(propertyPrefix),
            KData(new jderobot::pointCloudData()),
            syncController(syncController)
    {
        this->prop = ic->getProperties();
        this->dataPath = prop->getProperty(propertyPrefix + "Dir");
        this->initState = initStateIN;
        syncTask = new PointCloudSyncTask(this, this->dataPath);
        syncTask->start();
    }

    jderobot::pointCloudDataPtr ReplayerPointCloud::getCloudData(const Ice::Current &) {
        this->dataMutex.lock();
        jderobot::pointCloudDataPtr localData(KData);
        this->dataMutex.unlock();
        return localData;
    };

}