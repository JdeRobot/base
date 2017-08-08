//
// Created by frivas on 9/05/17.
//

#include "RGBDServer.h"
#include <jderobotutil/utils/CameraUtils.h>
#include <visionlib/colorspaces/imagecv.h>


namespace openniServer {

    RGBDServer::RGBDServer(std::string &propertyPrefix, const Ice::PropertiesPtr propIn,
                                       ConcurrentDevicePtr device) :
            prefix(propertyPrefix),
            device(device) {
        Ice::PropertiesPtr prop = propIn;
        mirror = prop->getPropertyAsIntWithDefault(prefix+"Mirror",0);
        int fps = prop->getPropertyAsIntWithDefault("openniServer.pointCloud.Fps", 10);
        bool extra = (bool) prop->getPropertyAsIntWithDefault("openniServer.ExtraCalibration", 0);
        replyCloud = new ReplyCloud(this, device, fps,mirror);
        this->control = replyCloud->start();
    }

    RGBDServer::~RGBDServer() {
        LOG(INFO) << "Stopping and joining thread for pointCloud";
        replyCloud->destroy();
        this->control.join();

    }

    jderobot::rgbData RGBDServer::getData(const Ice::Current &) {
        data = replyCloud->getData();
        return data;
    }



    RGBDServer::ReplyCloud::ReplyCloud(RGBDServer *server, ConcurrentDevicePtr device, int fpsIn,bool mirror):
            server(server),
            device(device),
            fps(fpsIn),
            _done(false),
            mirror(mirror)
    {
        this->cameraSize=cv::Size(device->getVideoMode().witdh,device->getVideoMode().heigth);
        LOG(INFO) << "Working with: " << device->getVideoMode().witdh << " x " <<device->getVideoMode().heigth;

    }

    void RGBDServer::ReplyCloud::run() {

        std::cout << "rgbd" << std::endl;
        while (this->cameraSize == cv::Size(0,0)){
            LOG(WARNING) << "Trying to get valid videoMode";

            this->cameraSize=cv::Size(device->getVideoMode().witdh,device->getVideoMode().heigth);
            sleep(1);
        }


        int cycle;


        cycle = (float) (1 / (float) fps) * 1000000;
        IceUtil::Time lastIT = IceUtil::Time::now();
        while (!(_done)) {
            float distance;
            cv::Mat rgb,depth;
            this->device->getSyncData(rgb,depth);

            if (mirror){
                cv::flip(rgb,rgb,1);
                cv::flip(depth,depth,1);
            }

            this->mutex.lock();
            jderobot::rgbData temporalData;
            temporalData.color = CameraUtils::convert(rgb);
            temporalData.depth = CameraUtils::convert(depth);

            this->stableData= temporalData;
            this->mutex.unlock();

            int delay = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();
            if (delay > cycle) {
                DLOG(WARNING) << "-------- openniServer: RGBD timeout";
            } else {
                if (delay < 1 || delay > cycle)
                    delay = 1;
                usleep(delay);
            }


            lastIT = IceUtil::Time::now();
        }
    }

    jderobot::rgbData RGBDServer::ReplyCloud::getData(){
        this->mutex.lock();
        this->returnData = this->stableData;
        this->mutex.unlock();
        return this->returnData;
    }

    void RGBDServer::ReplyCloud::destroy() {
        this->_done = true;
    }
}