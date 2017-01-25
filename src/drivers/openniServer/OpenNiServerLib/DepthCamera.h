//
// Created by frivas on 25/01/17.
//

#ifndef JDEROBOT_DEPTHCAMERA_H
#define JDEROBOT_DEPTHCAMERA_H

#include <jderobotutil/CameraHandler.h>
#include <jderobotutil/CameraTask.h>
#include <opencv2/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include "ConcurrentDevice.h"

namespace openniServer {
    class DepthCamera:  public jderobot::CameraHandler {
     public:
      DepthCamera(std::string propertyPrefix, Ice::CommunicatorPtr ic, ConcurrentDevicePtr device);
        ~DepthCamera();

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



#endif //JDEROBOT_DEPTHCAMERA_H
