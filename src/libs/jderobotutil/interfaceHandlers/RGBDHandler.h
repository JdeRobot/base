//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_RGBDHANDLER_H
#define JDEROBOT_RGBDHANDLER_H


#include <rgbd.h>
#include <Ice/Ice.h>
#include <opencv2/core/mat.hpp>

namespace jderobot {

    class RGBDHandler : virtual public jderobot::rgbd {
    public:
        RGBDHandler(std::string propertyPrefix, Ice::CommunicatorPtr ic);

        virtual ~RGBDHandler();

        void setData(const cv::Mat &colorImage, const cv::Mat &depthImage);
        virtual rgbData getData();


    private:
        Ice::PropertiesPtr prop;
        std::string prefix;
        IceUtil::Mutex mutex;
        cv::Mat colorImage;
        cv::Mat depthImage;
    };
}


#endif //JDEROBOT_RGBDHANDLER_H
