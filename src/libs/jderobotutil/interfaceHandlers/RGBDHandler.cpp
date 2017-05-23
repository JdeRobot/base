//
// Created by frivas on 30/04/17.
//

#include "RGBDHandler.h"

namespace jderobot {

    RGBDHandler::RGBDHandler(std::string propertyPrefix, Ice::CommunicatorPtr ic) {

    }


    RGBDHandler::~RGBDHandler() {

    }

    void RGBDHandler::setData(const cv::Mat &colorImage, const cv::Mat &depthImage) {
        this->mutex.lock();
        this->colorImage = colorImage.clone();
        this->depthImage = depthImage.clone();
        this->mutex.unlock();
    }

    rgbData RGBDHandler::getData() {
        rgbData data;
        this->mutex.lock();
        data.color->pixelData.resize(this->colorImage.rows*this->colorImage.cols * 3);
        memcpy(&(data.color->pixelData[0]),(unsigned char *) this->colorImage.data, this->colorImage.rows*this->colorImage.cols * 3);

        data.depth->pixelData.resize(this->depthImage.rows*this->depthImage.cols * 3);
        memcpy(&(data.depth->pixelData[0]),(unsigned char *) this->depthImage.data, this->depthImage.rows*this->depthImage.cols * 3);
        this->mutex.unlock();
        return data;

    }


}