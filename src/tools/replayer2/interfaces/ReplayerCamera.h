//
// Created by frivas on 27/04/17.
//

#ifndef JDEROBOT_REPLAYERCAMERA_H
#define JDEROBOT_REPLAYERCAMERA_H


#include <jderobotutil/interfaceHandlers/CameraHandler.h>
#include <jderobotutil/interfaceHandlers/CameraTask.h>
#include <logger/Logger.h>
#include <utils/SyncController.h>
#include <opencv2/imgcodecs.hpp>
#include "SyncTask.h"

namespace replayer {
    class ReplayerCamera : public jderobot::CameraHandler {
    public:
        ReplayerCamera(std::string propertyPrefix, Ice::CommunicatorPtr ic,replayer::SyncControllerPtr syncController,long long int initStateIN);
        void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr &cb, const std::string &format,const Ice::Current &c);


    private:


        /*************  CameraSyncTask ***********************/
        class CameraSyncTask: public SyncTask<ReplayerCamera*>{
        public:
            CameraSyncTask(ReplayerCamera* camera, std::string pathIn, std::string fileFormatIN):SyncTask(pathIn,"cameraData.jde",camera){
                this->path=pathIn;
                this->fileFormat=fileFormatIN;
            }

            virtual void generateData(){
                cv::Mat tempImage = cv::imread(this->path + lineData + "." + this->fileFormat);
                tempImage.copyTo(this->interface->image);
            }
        private:
            std::string fileFormat;
        };


        /*************  ReplyTask ***********************/
        class ReplyTask : public jderobot::CameraTask {
        public:
            ReplyTask(ReplayerCamera* camera, int fps):jderobot::CameraTask(camera,fps),mycamera(camera) {

            }

            virtual void createCustomImage(cv::Mat &image) {

                this->mycamera->dataMutex.lock();
                image = this->mycamera->image.clone();
                this->mycamera->dataMutex.unlock();
            }
            ReplayerCamera* mycamera;
        };


        typedef IceUtil::Handle <ReplyTask> ReplyTaskPtr;
        ReplyTaskPtr replyTask;


    bool startThread;
    Ice::PropertiesPtr prop;
    cv::Mat image;
    int width;
    int height;
    typedef IceUtil::Handle<CameraSyncTask> SyncTaskPtr;
    SyncTaskPtr syncTask;
    std::string fileFormat;
    std::string dataPath;
    std::string format;

    public:
        SyncControllerPtr syncController;
        long long int initState;
        IceUtil::Mutex dataMutex;

    };
}
#endif //JDEROBOT_REPLAYERCAMERA_H
