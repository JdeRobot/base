//
// Created by frivas on 21/05/17.
//

#ifndef JDEROBOT_RGBDRINGNODE_H
#define JDEROBOT_RGBDRINGNODE_H

#include <opencv2/core/core.hpp>

namespace RingBufferNS{
    struct RGBDRingNode {
        static void write(const std::string&  logPath, const std::string&  nameLog, std::vector<RingBufferNS::RGBDRingNode> data2save);

        long long int relativeTime;
        cv::Mat depth;
        cv::Mat rgb;

        int cameraId;
        std::vector<int> mCompression;
    };
}


#endif //JDEROBOT_RGBDRINGNODE_H
