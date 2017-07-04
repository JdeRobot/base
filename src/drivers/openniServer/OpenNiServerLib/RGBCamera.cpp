//
// Created by frivas on 24/01/17.
//

#include <logger/Logger.h>
#include "RGBCamera.h"
#include <opencv2/opencv.hpp>


namespace openniServer{
    RGBCamera::RGBCamera(std::string propertyPrefix, Ice::CommunicatorPtr ic,ConcurrentDevicePtr device) : CameraHandler(propertyPrefix, ic),device(device) {

        framerateN = prop->getPropertyAsIntWithDefault(prefix+"fps",25);
        mirror = prop->getPropertyAsIntWithDefault(prefix+"Mirror",0);
        std::string fmtStr = prop->getPropertyWithDefault(prefix+"Format","YUY2");//default format YUY2
        imageFmt = colorspaces::Image::Format::searchFormat(fmtStr);
        if (!imageFmt)
            LOG(WARNING) <<  "Format " + fmtStr + " unknown" ;
        imageDescription->size = imageDescription->width * imageDescription->height * CV_ELEM_SIZE(imageFmt->cvType);
        imageDescription->format = imageFmt->name;

        LOG(INFO) <<  "Starting thread for camera: " + cameraDescription->name ;
        replyTask = new ReplyTask(this, framerateN, device,mirror);

        this->control=replyTask->start();//my own thread

    }

    void RGBCamera::getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr &cb, const std::string &format,
                                       const Ice::Current &c) {
        replyTask->pushJob(cb, format);

    }

    RGBCamera::~RGBCamera() {
        LOG(INFO) <<  "Stopping and joining thread for color camera" ;
        replyTask->destroy();
        this->control.join();
    }

    RGBCamera::ReplyTask::ReplyTask(const jderobot::Camera *camera, int fps,ConcurrentDevicePtr device, bool mirror) : CameraTask(camera, fps),device(device),mirror(mirror) {

    }

    void RGBCamera::ReplyTask::createCustomImage(cv::Mat &image) {
        image=this->device->getRGBImage();
        if (mirror){
            cv::flip(image,image,1);
        }
    }

}
