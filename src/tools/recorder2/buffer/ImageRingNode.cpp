//
// Created by frivas on 21/05/17.

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <opencv2/imgcodecs.hpp>
#include "ImageRingNode.h"
#include <logger/Logger.h>

namespace RingBufferNS {
    void ImageRingNode::write(const std::string&  logPath, const std::string &nameLog, std::vector<ImageRingNode> data2save) {
        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();
        std::stringstream root_path;
        root_path << "data-" << nameLog << "/" <<  logPath << "/";
        std::string jdeFile = root_path.str() + "cameraData2.jde";
        std::ofstream myfile;
        myfile.open (jdeFile);




        for (auto it = data2save.begin(); it < data2save.end(); it++) {
            std::stringstream path;
            path << root_path.str() << it->relativeTime << ".png";
            std::cout << "saving: " << path.str() << std::endl;
            cv::imwrite(path.str(), it->frame, it->mCompression);
            myfile << it->relativeTime << "/n";

            std::cout << it->relativeTime << " to: " << jdeFile << std::endl;
        }
        myfile.close();
        boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();

        boost::posix_time::time_duration total = end - init;
        LOG(INFO) << "Total milliseconds spent: " << total.total_milliseconds() << " - " << "Total Size: "
                  << data2save.size() << std::endl;

        for (auto it = data2save.begin(); it < data2save.end(); it++)
            it->frame.release();
    }

}