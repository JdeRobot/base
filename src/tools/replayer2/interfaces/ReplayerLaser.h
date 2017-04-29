//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_REPLAYERLASER_H
#define JDEROBOT_REPLAYERLASER_H


#include <jderobot/logger/Logger.h>
#include "SyncTask.h"
#include <jderobot/laser.h>
#include <Ice/Ice.h>
#include <utils/SyncController.h>
#include <laser.h>


namespace replayer {

    class ReplayerLaser : virtual public jderobot::Laser {
    public:
        ReplayerLaser (std::string& propertyPrefix, Ice::CommunicatorPtr ic,replayer::SyncControllerPtr syncController, long long int initStateIN):
                prefix(propertyPrefix),
                KData(new jderobot::LaserData()),
                syncController(syncController)
        {
            this->prop = ic->getProperties();
            this->dataPath=prop->getProperty(propertyPrefix+"Dir");
            this->initState=initStateIN;
            syncTask  = new LaserSyncTask(this,this->dataPath);
            syncTask->start();
        }
        virtual jderobot::LaserDataPtr getLaserData(const Ice::Current&){
            this->dataMutex.lock();
            jderobot::LaserDataPtr localData(KData);
            localData->numLaser=180;
            this->dataMutex.unlock();
            return localData;
        };
    private:

        class LaserSyncTask : public SyncTask<ReplayerLaser*>{
        public:
            LaserSyncTask(ReplayerLaser* laser, std::string pathIn):
                    SyncTask(pathIn,"laserData.jde",laser),
                    myLaser(laser)
            {
                this->path=pathIn;
            }

            virtual void generateData(){
                long long int relative;
                int sizeVector;
                int maxRange;
                int minRange;
                float maxAngle;
                float minAngle;

                std::istringstream sTemp(lineData);
                sTemp >> relative;
                sTemp >> sizeVector;

                sTemp >> maxRange;
                sTemp >> minRange;
                sTemp >> maxAngle;
                sTemp >> minAngle;

                std::ostringstream relativeFile;
                relativeFile << relative;
                std::string localFile(this->path + relativeFile.str());

                std::ifstream infile(localFile.c_str(), std::ios::in | std::ios::binary);
                this->myLaser->KData->distanceData.resize(sizeVector);
                this->myLaser->KData->minAngle = minAngle;
                this->myLaser->KData->maxAngle = maxAngle;
                this->myLaser->KData->minRange = minRange;
                this->myLaser->KData->maxRange = maxRange;
                infile.read((char *)&this->myLaser->KData->distanceData.front(), this->myLaser->KData->distanceData.size()*sizeof(int));

            }
        private:
            ReplayerLaser* myLaser;
        };

        typedef IceUtil::Handle <LaserSyncTask> SyncTaskPtr;
        SyncTaskPtr syncTask;
        std::string prefix;
        jderobot::LaserDataPtr KData;
        Ice::PropertiesPtr prop;
        std::string dataPath;

    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;


    };

}


#endif //JDEROBOT_REPLAYERLASER_H
