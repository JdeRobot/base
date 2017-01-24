//
// Created by frivas on 23/01/17.
//

#ifndef OPENNISERVER_DEPTHOPENCVCONVERTER_H
#define OPENNISERVER_DEPTHOPENCVCONVERTER_H

#define MAX_LENGHT 10000

#include <OpenNI.h>
#include <opencv2/opencv.hpp>

class OpenCVConverter {
public:
    static cv::Mat convertDepthToCVMat(const openni::DepthPixel *depthPixels, int strideInBytes, cv::Size imageSize, std::vector<int> &distances);
    static cv::Mat convertRGBToCVMat(const openni::RGB888Pixel* pImageRow, int strideInBytes, cv::Size imageSize);
};


#endif //OPENNISERVER_DEPTHOPENCVCONVERTER_H
