//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_REPLAYERENCODERS_H
#define JDEROBOT_REPLAYERENCODERS_H


#include <logger/Logger.h>
#include "SyncTask.h"
#include <Ice/Ice.h>
#include <utils/SyncController.h>
#include <laser.h>
#include <encoders.h>

namespace  replayer {
    class ReplayerEncoders : virtual public jderobot::Encoders {
    public:
        ReplayerEncoders(std::string &propertyPrefix, Ice::CommunicatorPtr ic,SyncControllerPtr syncController, long long int initStateIN);

        virtual jderobot::EncodersDataPtr getEncodersData(const Ice::Current &);

    private:

        class EncodersSyncTask : public SyncTask<ReplayerEncoders*>{
        public:
            EncodersSyncTask(ReplayerEncoders* encoders, std::string pathIn):
                    SyncTask(pathIn,"encoderData.jde",encoders),
                    myEncoder(encoders)
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
                infile.read((char *)&this->myEncoder->tempData, sizeof(encoders));
                this->myEncoder->encData->robotx=this->myEncoder->tempData.robotx;
                this->myEncoder->encData->roboty=this->myEncoder->tempData.roboty;
                this->myEncoder->encData->robottheta=this->myEncoder->tempData.robottheta;
                this->myEncoder->encData->robotcos=this->myEncoder->tempData.robotcos;
                this->myEncoder->encData->robotsin=this->myEncoder->tempData.robotsin;

            }
        private:
            ReplayerEncoders* myEncoder;
        };

        struct encoders {
            float robotx;
            float roboty;
            float robottheta;
            float robotcos;
            float robotsin;
        };

        typedef IceUtil::Handle<EncodersSyncTask> SyncTaskPtr;
        SyncTaskPtr v;
        std::string prefix;
        jderobot::EncodersDataPtr encData;
        Ice::PropertiesPtr prop;
        std::string dataPath;
        IceUtil::Mutex m;
        encoders tempData;

    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;


    };
}
#endif //JDEROBOT_REPLAYERENCODERS_H
