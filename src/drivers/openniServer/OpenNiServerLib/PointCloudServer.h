//
// Created by frivas on 25/01/17.
//

#ifndef JDEROBOT_POINTCLOUDSERVER_H
#define JDEROBOT_POINTCLOUDSERVER_H

#include <jderobot/pointcloud.h>
#include <Ice/Properties.h>
#include <logger/Logger.h>
#include "myprogeo.h"
#include "ConcurrentDevice.h"

namespace openniServer {

    class PointCloudServer : virtual public jderobot::pointCloud {
    public:
        PointCloudServer(std::string &propertyPrefix, const Ice::PropertiesPtr propIn,ConcurrentDevicePtr device);

        virtual ~PointCloudServer();


        virtual jderobot::pointCloudDataPtr getCloudData(const Ice::Current &);

    private:
        class ReplyCloud : public IceUtil::Thread {
        public:
            ReplyCloud(PointCloudServer *pcloud,ConcurrentDevicePtr device, std::string calibration_filepath, int fpsIn, bool extra_calibration);

            void run();


            jderobot::pointCloudDataPtr getCloud();

            virtual void destroy();


        private:
            PointCloudServer *myCloud;
            ConcurrentDevicePtr device;
            myprogeo *mypro;
            std::string calibration_filepath;
            int fps;
            jderobot::pointCloudDataPtr temporalData, stableData, returnData;
            cv::Size cameraSize;
            jderobot::RGBPoint auxP;
            bool withExtraCalibration;
            bool _done;
            IceUtil::Mutex mutex;


        };

        typedef IceUtil::Handle<ReplyCloud> ReplyCloudPtr;
        ReplyCloudPtr replyCloud;
        std::string prefix;
        jderobot::pointCloudDataPtr data;
        IceUtil::ThreadControl control;
        ConcurrentDevicePtr device;


    };

#endif //JDEROBOT_POINTCLOUDSERVER_H
}
