//
// Created by frivas on 9/05/17.
//

#ifndef JDEROBOT_RGBDSERVER_H
#define JDEROBOT_RGBDSERVER_H

#include <Ice/Ice.h>
#include "ConcurrentDevice.h"
#include <jderobot/rgbd.h>


namespace openniServer {

    class RGBDServer: virtual public jderobot::rgbd {
    public:
        RGBDServer(std::string &propertyPrefix, const Ice::PropertiesPtr propIn, ConcurrentDevicePtr device);

        virtual ~RGBDServer();


        virtual jderobot::rgbData getData(const Ice::Current &);

    private:
        class ReplyCloud : public IceUtil::Thread {
        public:
            ReplyCloud(RGBDServer *server, ConcurrentDevicePtr device, int fpsIn);

            void run();


            jderobot::rgbData getData();

            virtual void destroy();


        private:
            RGBDServer *server;
            ConcurrentDevicePtr device;
            int fps;
            jderobot::rgbData stableData, returnData;
            cv::Size cameraSize;
            bool _done;
            IceUtil::Mutex mutex;
        };

        typedef IceUtil::Handle <ReplyCloud> ReplyCloudPtr;
        ReplyCloudPtr replyCloud;
        std::string prefix;
        jderobot::rgbData data;
        IceUtil::ThreadControl control;
        ConcurrentDevicePtr device;

    };

}

#endif //JDEROBOT_RGBDSERVER_H
