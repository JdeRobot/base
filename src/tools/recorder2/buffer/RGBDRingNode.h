//
// Created by frivas on 21/05/17.
//

#ifndef JDEROBOT_RGBDRINGNODE_H
#define JDEROBOT_RGBDRINGNODE_H

#include <opencv2/core/mat.hpp>

namespace RingBufferNS {

    class RGBDRingNode {
        class ImageRingNode {
        public:
            static void write(const std::string &nameLog, std::vector<RingBufferNS::RGBDRingNode> data2save);

//    private:
            long long int relativeTime;
            cv::Mat frame;
            int cameraId;
            std::vector<int> mCompression;


        };
    };
}


#endif //JDEROBOT_RGBDRINGNODE_H
