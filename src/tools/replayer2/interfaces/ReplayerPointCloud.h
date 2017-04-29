//
// Created by frivas on 29/04/17.
//

#ifndef JDEROBOT_REPLAYERPOINTCLOUD_H
#define JDEROBOT_REPLAYERPOINTCLOUD_H


#include <jderobot/pointcloud.h>
#include <jderobot/logger/Logger.h>
#include "SyncTask.h"
#include <Ice/Ice.h>
#include <utils/SyncController.h>


namespace replayer {

    class ReplayerPointCloud : virtual public jderobot::pointCloud {
    public:
        ReplayerPointCloud(std::string &propertyPrefix, Ice::CommunicatorPtr ic,replayer::SyncControllerPtr syncController, long long int initStateIN);

        virtual jderobot::pointCloudDataPtr getCloudData(const Ice::Current &);

    private:

        class PointCloudSyncTask : public SyncTask<ReplayerPointCloud*>{
        public:
            PointCloudSyncTask(ReplayerPointCloud* pointCloud, std::string pathIn):
                    SyncTask(pathIn,"pointCloudData.jde",pointCloud),
                    myPointCloud(pointCloud)
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
                this->myPointCloud->KData->p.resize(sizeVector);
                infile.read((char *)&this->myPointCloud->KData->p.front(), this->myPointCloud->KData->p.size()*sizeof(jderobot::RGBPoint));

            }
        private:
            ReplayerPointCloud* myPointCloud;
        };

        typedef IceUtil::Handle <PointCloudSyncTask> SyncTaskPtr;
        SyncTaskPtr syncTask;
        std::string prefix;
        jderobot::pointCloudDataPtr KData;
        Ice::PropertiesPtr prop;
        std::string dataPath;

    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;


    };

}
#endif //JDEROBOT_REPLAYERPOINTCLOUD_H

