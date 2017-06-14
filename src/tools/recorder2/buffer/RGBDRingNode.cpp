//
// Created by frivas on 21/05/17.
//

#include "RGBDRingNode.h"
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <logger/Logger.h>
#include <boost/algorithm/string/replace.hpp>


namespace RingBufferNS {

    void RGBDRingNode::write(const std::string&  logPath,const std::string &nameLog, std::vector<RingBufferNS::RGBDRingNode> data2save) {
        boost::posix_time::ptime init = boost::posix_time::microsec_clock::local_time();
        std::string root_path;
        root_path = "data-" + nameLog + "/" +  logPath + "/";

        boost::replace_all(root_path, "./", "");
        boost::replace_all(root_path, "//", "/");

//        std::string jdeFile = root_path.str() + "cameraData2.jde";
//        std::ofstream myfile;
//        myfile.open (jdeFile);

        for (auto it = data2save.begin(); it < data2save.end(); it++) {
            std::stringstream pathRGB;
            pathRGB << root_path << it->relativeTime << "rgb.png";
            cv::imwrite(pathRGB.str(), it->rgb, it->mCompression);
            std::stringstream pathDEPTH;
            pathDEPTH << root_path << it->relativeTime << "depth.png";
            cv::imwrite(pathDEPTH.str(), it->depth, it->mCompression);
        }


        boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();

        boost::posix_time::time_duration total = end - init;
        LOG(INFO) << "Total milliseconds spent: " << total.total_milliseconds() << " - " << "Total Size: "
                  << data2save.size() << std::endl;

        for (auto it = data2save.begin(); it < data2save.end(); it++) {
            it->depth.release();
            it->rgb.release();
        }
    }
}
