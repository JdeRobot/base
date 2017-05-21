//
// Created by frivas on 21/05/17.
//

#ifndef JDEROBOT_IMAGERINGNODE_H
#define JDEROBOT_IMAGERINGNODE_H

#include <opencv2/core/mat.hpp>

namespace RingBufferNS{
    class ImageRingNode {
    public:
        static void write(const std::string&  nameLog, std::vector<RingBufferNS::ImageRingNode> data2save);

//    private:
        long long int relativeTime;
        cv::Mat frame;
        int cameraId;
        std::vector<int> mCompression;


    };
}

#endif //JDEROBOT_IMAGERINGNODE_H
