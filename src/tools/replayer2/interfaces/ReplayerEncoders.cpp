//
// Created by frivas on 30/04/17.
//

#include "ReplayerEncoders.h"


namespace replayer{

    ReplayerEncoders::ReplayerEncoders(std::string &propertyPrefix, Ice::CommunicatorPtr ic,
                                       SyncControllerPtr syncController, long long int initStateIN) :
            prefix(propertyPrefix),
            encData(new jderobot::EncodersData()),
            syncController(syncController)
    {
        this->prop = ic->getProperties();
        this->dataPath = prop->getProperty(propertyPrefix + "Dir");
        this->initState = initStateIN;
        v = new EncodersSyncTask(this, this->dataPath);
        v->start();
    }

    jderobot::EncodersDataPtr ReplayerEncoders::getEncodersData(const Ice::Current &) {
        //check si los dos son iguales
        this->m.lock();
        jderobot::EncodersDataPtr localData(encData);
        this->m.unlock();
        return localData;
    };
}