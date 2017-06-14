//
// Created by frivas on 21/05/17.
//

#ifndef JDEROBOT_RECORDERRGBD_H
#define JDEROBOT_RECORDERRGBD_H


#include <rgbd.h>
#include <opencv2/core/mat.hpp>
#include <boost/shared_ptr.hpp>

namespace recorder{
    struct RecorderRGBD {
        cv::Mat rgb;
        cv::Mat depth;
        RecorderRGBD(const jderobot::rgbData& data);
        RecorderRGBD(const RecorderRGBD& other);
        RecorderRGBD();
    };

    typedef boost::shared_ptr<RecorderRGBD> RecorderRGBDPtr;
}

#endif //JDEROBOT_RECORDERRGBD_H
