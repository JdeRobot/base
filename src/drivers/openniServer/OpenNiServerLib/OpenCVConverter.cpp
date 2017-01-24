//
// Created by frivas on 23/01/17.
//

#include "OpenCVConverter.h"

cv::Mat OpenCVConverter::convertDepthToCVMat(const openni::DepthPixel *depthPixels, int strideInBytes, cv::Size imageSize, std::vector<int> &distances) {
    cv::Mat src(imageSize,CV_8UC3,cv::Scalar(0,0,0));

    int restOfRow = strideInBytes / sizeof(openni::DepthPixel) - imageSize.width;

    for (int y = 0; y < imageSize.height; ++y)
    {
        for (int x = 0; x < imageSize.width; ++x, ++depthPixels)
        {
            distances[(y*imageSize.width + x)] = *depthPixels;
            if (*depthPixels != 0)
            {
                src.data[(y*imageSize.width+ x)*3+0] = (float(*depthPixels)/(float)MAX_LENGHT)*255.;
                src.data[(y*imageSize.width+ x)*3+1] = (*depthPixels)>>8;
                src.data[(y*imageSize.width+ x)*3+2] = (*depthPixels)&0xff;
            }

        }

        depthPixels += restOfRow;
    }
    return src;

}

cv::Mat OpenCVConverter::convertRGBToCVMat(const openni::RGB888Pixel *pImageRow, int strideInBytes, cv::Size imageSize) {
    cv::Mat src(imageSize,CV_8UC3);

    int rowSize = strideInBytes / sizeof(openni::RGB888Pixel);



    for (int y = 0; y < imageSize.height; ++y)
    {
        const openni::RGB888Pixel* pImage = pImageRow;
        for (int x = 0; x < imageSize.width; ++x, ++pImage)
        {
            src.data[(y*imageSize.width + x)*3 + 0] = pImage->r;
            src.data[(y*imageSize.width + x)*3 + 1] = pImage->g;
            src.data[(y*imageSize.width + x)*3 + 2] = pImage->b;
        }
        pImageRow += rowSize;
    }

    return src;

}
