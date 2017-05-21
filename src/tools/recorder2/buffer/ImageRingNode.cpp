//
// Created by frivas on 21/05/17.

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ImageRingNode.h"
#include <logger/Logger.h>

namespace RingBufferNS {
    void ImageRingNode::write(const std::string &nameLog, std::vector<ImageRingNode> data2save) {
        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();
        for (auto it = data2save.begin(); it < data2save.end(); it++) {
            std::stringstream path;
            path << "data-" << nameLog << "/images/camera" << it->cameraId << "/" << it->relativeTime << ".png";
            cv::imwrite(path.str(), it->frame, it->mCompression);
        }
        boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();

        boost::posix_time::time_duration total = end - init;
        LOG(INFO) << "Total milliseconds spent: " << total.total_milliseconds() << " - " << "Total Size: "
                  << data2save.size() << std::endl;

        for (auto it = data2save.begin(); it < data2save.end(); it++)
            it->frame.release();
    }

}