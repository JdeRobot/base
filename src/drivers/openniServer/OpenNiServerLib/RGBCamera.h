//
// Created by frivas on 24/01/17.
//

#ifndef JDEROBOT_RGBCAMERA_DEVICE_H
#define JDEROBOT_RGBCAMERA_DEVICE_H

#include <jderobotutil/CameraHandler.h>
#include <jderobotutil/CameraTask.h>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "ConcurrentDevice.h"

namespace openniServer {
    class RGBCamera:  public jderobot::CameraHandler {
     public:
      RGBCamera(std::string propertyPrefix, Ice::CommunicatorPtr ic, ConcurrentDevicePtr device);

      void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, const std::string& format, const Ice::Current& c);



     private:
        class ReplyTask: public jderobot::CameraTask {
        public:
            ReplyTask(const jderobot::Camera* camera, int fps,ConcurrentDevicePtr device);

            virtual void createCustomImage(cv::Mat& image);

        private:
            ConcurrentDevicePtr device;
        };



        typedef IceUtil::Handle<ReplyTask> ReplyTaskPtr;
        ReplyTaskPtr replyTask;
        ConcurrentDevicePtr device;
        int framerateN;
        IceUtil::ThreadControl control;

    };


}


#endif //JDEROBOT_RGBCAMERA_DEVICE_H
