//
// Created by frivas on 27/04/17.
//

#include "ReplayerCamera.h"

namespace replayer {
    ReplayerCamera::ReplayerCamera(std::string propertyPrefix, Ice::CommunicatorPtr ic,
                                             replayer::SyncControllerPtr syncController, long long int initStateIN) :
            jderobot::CameraHandler(propertyPrefix,ic),
            syncController(syncController)

    {
        imageDescription = (new jderobot::ImageDescription());
        prop = ic->getProperties();
        cameraDescription = (new jderobot::CameraDescription());
        imageDescription->width=prop->getPropertyAsIntWithDefault(propertyPrefix + "ImageWidth",320);
        imageDescription->height=prop->getPropertyAsIntWithDefault(propertyPrefix + "ImageHeight",240);
        this->dataPath=prop->getProperty(propertyPrefix+"Dir");
        this->fileFormat=prop->getProperty(propertyPrefix+"FileFormat");
        imageDescription->format = prop->getProperty(propertyPrefix+"Format");
        imageDescription->size = width*height*3;

        LOG(INFO)<< "PATH " + this->dataPath ;
        LOG(INFO)<< "FORMAT: " + this->fileFormat ;


        this->initState=initStateIN;
        //sync task
        syncTask = new CameraSyncTask(this,this->dataPath, this->fileFormat);
        syncTask->start();
        //reply task
        replyTask = new ReplyTask(this,30); //30 fps ~ real time
        replyTask->start(); // my own thread

    }

    void
    ReplayerCamera::getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr &cb, const std::string &format,
                                       const Ice::Current &c) {
        replyTask->pushJob(cb, format);
    }

}