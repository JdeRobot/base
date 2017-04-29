//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_REPLAYERPOSE3DENCODERS_H
#define JDEROBOT_REPLAYERPOSE3DENCODERS_H

#include <jderobot/logger/Logger.h>
#include "SyncTask.h"
#include <jderobot/pose3dencoders.h>
#include <Ice/Ice.h>
#include <utils/SyncController.h>
#include <laser.h>

namespace replayer {
    class ReplayerPose3DEncoders : virtual public jderobot::Pose3DEncoders {
    public:
        ReplayerPose3DEncoders(std::string &propertyPrefix, Ice::CommunicatorPtr ic,
                               replayer::SyncControllerPtr syncController, long long int initStateIN);

        virtual jderobot::Pose3DEncodersDataPtr getPose3DEncodersData(const Ice::Current &);

    private:



        class Pose3DEncodersSyncTask : public SyncTask<ReplayerPose3DEncoders*>{
        public:
            Pose3DEncodersSyncTask(ReplayerPose3DEncoders* pose3DEncoders, std::string pathIn):
                    SyncTask(pathIn,"pose3dencoderData.jde",pose3DEncoders),
                    myPose3DEncoders(pose3DEncoders)
            {
                this->path=pathIn;
            }

            virtual void generateData(){
                long long int relative;
                int sizeVector;

                std::istringstream sTemp(lineData);
                sTemp >> relative;
                sTemp >> sizeVector;

                std::ostringstream relativeFile;
                relativeFile << relative;
                std::string localFile(this->path + relativeFile.str());

                std::ifstream infile(localFile.c_str(), std::ios::in | std::ios::binary);
                infile.read((char *) &this->myPose3DEncoders->tempData, sizeof(pose3dencoders));
                this->myPose3DEncoders->encData->x = this->myPose3DEncoders->tempData.x;
                this->myPose3DEncoders->encData->y = this->myPose3DEncoders->tempData.y;
                this->myPose3DEncoders->encData->z = this->myPose3DEncoders->tempData.z;
                this->myPose3DEncoders->encData->pan = this->myPose3DEncoders->tempData.pan;
                this->myPose3DEncoders->encData->tilt = this->myPose3DEncoders->tempData.tilt;
                this->myPose3DEncoders->encData->roll = this->myPose3DEncoders->tempData.roll;
                this->myPose3DEncoders->encData->maxPan = this->myPose3DEncoders->tempData.maxPan;
                this->myPose3DEncoders->encData->maxTilt = this->myPose3DEncoders->tempData.maxTilt;
                this->myPose3DEncoders->encData->minPan = this->myPose3DEncoders->tempData.minPan;
                this->myPose3DEncoders->encData->minTilt = this->myPose3DEncoders->tempData.minTilt;

            }
        private:
            ReplayerPose3DEncoders* myPose3DEncoders;
        };

        struct pose3dencoders {
            float x;
            float y;
            float z;
            float pan;
            float tilt;
            float roll;
            int clock;
            float maxPan;
            float maxTilt;
            float minPan;
            float minTilt;
        };

        typedef IceUtil::Handle<Pose3DEncodersSyncTask> SyncTaskPtr;
        SyncTaskPtr v;
        std::string prefix;
        jderobot::Pose3DEncodersDataPtr encData;
        Ice::PropertiesPtr prop;
        pose3dencoders tempData;
        std::string dataPath;


    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;


    };
}

#endif //JDEROBOT_REPLAYERPOSE3DENCODERS_H
