//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_REPLAYERPOSE3D_H
#define JDEROBOT_REPLAYERPOSE3D_H


#include <logger/Logger.h>
#include "SyncTask.h"
#include <jderobot/pose3d.h>
#include <Ice/Ice.h>
#include <utils/SyncController.h>

namespace replayer {
    class ReplayerPose3D : virtual public jderobot::Pose3D {
    public:
        ReplayerPose3D(std::string &propertyPrefix, Ice::CommunicatorPtr ic, SyncControllerPtr syncController,long long int initStateIN);

        virtual jderobot::Pose3DDataPtr getPose3DData(const Ice::Current &);

        virtual Ice::Int setPose3DData(const jderobot::Pose3DDataPtr &, const Ice::Current &) {
            return 0;
        };

    private:


        class Pose3DEncodersSyncTask : public SyncTask<ReplayerPose3D *> {
        public:
            Pose3DEncodersSyncTask(ReplayerPose3D *pose3D, std::string pathIn) :
                    SyncTask(pathIn, "pose3dData.jde", pose3D),
                    myPose3d(pose3D) {
                this->path = pathIn;
            }

            virtual void generateData() {
                long long int relative;
                int sizeVector;

                std::istringstream sTemp(lineData);
                sTemp >> relative;
                sTemp >> sizeVector;

                std::ostringstream relativeFile;
                relativeFile << relative;
                std::string localFile(this->path + relativeFile.str());

                std::ifstream infile(localFile.c_str(), std::ios::in | std::ios::binary);
                infile.read((char *) &this->myPose3d->tempData, sizeof(pose3d));
                //hago la copia al interfaz....
                this->myPose3d->pose3dData->x = this->myPose3d->tempData.x;
                this->myPose3d->pose3dData->y = this->myPose3d->tempData.y;
                //std::cout<<localFile.c_str()<<", y:"<<this->myPose3d->tempData.y<<std::endl;
                this->myPose3d->pose3dData->z = this->myPose3d->tempData.z;
                this->myPose3d->pose3dData->h = this->myPose3d->tempData.h;
                this->myPose3d->pose3dData->q0 = this->myPose3d->tempData.q0;
                this->myPose3d->pose3dData->q1 = this->myPose3d->tempData.q1;
                this->myPose3d->pose3dData->q2 = this->myPose3d->tempData.q2;
                this->myPose3d->pose3dData->q3 = this->myPose3d->tempData.q3;

            }

        private:
            ReplayerPose3D *myPose3d;
        };

        struct pose3d {
            float x;
            float y;
            float z;
            float h;
            float q0;
            float q1;
            float q2;
            float q3;
        };

        typedef IceUtil::Handle<Pose3DEncodersSyncTask> SyncTaskPtr;
        SyncTaskPtr v;
        std::string prefix;
        jderobot::Pose3DDataPtr pose3dData;
        Ice::PropertiesPtr prop;
        std::string dataPath;
        pose3d tempData;

    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;


    };

}

#endif //JDEROBOT_REPLAYERPOSE3D_H
