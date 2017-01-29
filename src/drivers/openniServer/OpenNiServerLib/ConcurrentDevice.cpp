//
// Created by frivas on 23/01/17.
//

#include "ConcurrentDevice.h"
#include "OpenNITypes.h"
#include <OpenNiServerLib/OpenCVConverter.h>
#include <boost/lexical_cast.hpp>
#include <logger/Logger.h>

ConcurrentDevice::ConcurrentDevice(int fps, int cameraIdx, DeviceConfig config, int mode):componentAlive(true),fps(fps),cameraIdx(cameraIdx),config(config),mode(mode),g_bIsDepthOn(false),g_bIsColorOn(false),g_bIsIROn(false),g_depthSensorInfo(NULL),g_colorSensorInfo(NULL),g_irSensorInfo(NULL)     {

    openni::Status nRetVal = openni::OpenNI::initialize();
    if (nRetVal != openni::STATUS_OK)
    {
        jderobot::Logger::getInstance()->error("Cannot initialize openni");
        return;
    }

    // Register to OpenNI events.
    static OpenNIDeviceListener deviceListener(&g_device);

    openni::OpenNI::addDeviceDisconnectedListener(&deviceListener);
    openni::OpenNI::addDeviceStateChangedListener(&deviceListener);


    // Register to OpenNI events.
    openni::Array<openni::DeviceInfo> deviceList;
    openni::OpenNI::enumerateDevices(&deviceList);

    for (int i = 0; i < deviceList.getSize(); ++i)
    {
        jderobot::Logger::getInstance()->info("[" + boost::lexical_cast<std::string>(i) + "]" + deviceList[i].getName() + " [" +  deviceList[i].getVendor() + "] " + "(" + deviceList[i].getUri() + ")");
    }



    //checking the number off connected devices
    if (deviceList.getSize() < 1)
    {
        jderobot::Logger::getInstance()->error( "Missing devices" );
        openni::OpenNI::shutdown();
    }


    deviceUri = deviceList[cameraIdx].getUri();


    // Open the requested device.
    nRetVal = g_device.open(deviceUri);
    if (nRetVal != openni::STATUS_OK)
    {
        jderobot::Logger::getInstance()->error("Error openning device");
        return;
    }

//    config.openColor=SENSOR_ON;
//    config.openDepth=SENSOR_ON;
//    config.openIR=SENSOR_OFF;


    if (0 != openCommon())
    {
        jderobot::Logger::getInstance()->error("Error openning streams");
    }

//    this->changeRegistration(1); image registered over rgb
    this->changeRegistration(this->config.registrationMode);

    cycle=(float)(1/(float)fps)*1000000;


    jderobot::Logger::getInstance()->info("Device initialized");
    return;


}
ConcurrentDevice::~ConcurrentDevice() {

}


void ConcurrentDevice::run() {

    IceUtil::Time lastIT=IceUtil::Time::now();
    while(this->componentAlive) {
        readFrame();


        int process = IceUtil::Time::now().toMicroSeconds() - lastIT.toMicroSeconds();

        if (process > (int)cycle ){
            jderobot::Logger::getInstance()->warning("-------- openniServer: RGB openni timeout-" );
        }
        else{
            int delay = (int)cycle - process;
            if (delay <1 || delay > (int)cycle)
                delay = 1;

            usleep(delay);
        }
        lastIT=IceUtil::Time::now();


//        cv::Mat depthImage= this->getDepthImage();
//        if (!depthImage.empty())
//            cv::imshow("DepthImage", this->getDepthImage());
//        cv::Mat colorImage=this->getRGBImage();
//        if (!colorImage.empty())
//            cv::imshow("ColorImage", this->getRGBImage());
//
//        cv::waitKey(20);
    }
}


int ConcurrentDevice::openStream(const char* name, openni::SensorType sensorType, SensorOpenType openType, openni::VideoStream& stream, const openni::SensorInfo** ppSensorInfo, bool* pbIsStreamOn)
{
    *ppSensorInfo = g_device.getSensorInfo(sensorType);
    *pbIsStreamOn = false;

    if (openType == SENSOR_OFF)
    {
        return 0;
    }

    if (*ppSensorInfo == NULL)
    {
        if (openType == SENSOR_ON)
        {
            jderobot::Logger::getInstance()->error("No "  + std::string(name) + " sensor available");
            return -1;
        }
        else
        {
            return 0;
        }
    }

    openni::Status nRetVal = stream.create(g_device, sensorType);
    if (nRetVal != openni::STATUS_OK)
    {
        if (openType == SENSOR_ON)
        {
            jderobot::Logger::getInstance()->error("Failed to create " + std::string(openni::OpenNI::getExtendedError()) + " " + std::string(name));
            return -2;
        }
        else
        {
            return 0;
        }
    }

    nRetVal = stream.start();
    if (nRetVal != openni::STATUS_OK)
    {
        stream.destroy();

        if (openType == SENSOR_ON)
        {
            jderobot::Logger::getInstance()->error("Failed to start depth stream: " + std::string(openni::OpenNI::getExtendedError()));
            return -3;
        }
        else
        {
            return 0;
        }
    }


    if ((*ppSensorInfo)->getSupportedVideoModes().getSize()==1){
        jderobot::Logger::getInstance()->warning("Only one mode is avialable image will be post processed is the mode does not match with the device output");
        this->videoMode= VideoMode(this->mode);
    }
    else{
        nRetVal = stream.setVideoMode((*ppSensorInfo)->getSupportedVideoModes()[this->mode]);
        if (nRetVal != openni::STATUS_OK)
        {
            stream.destroy();

            if (openType == SENSOR_ON)
            {
                jderobot::Logger::getInstance()->error("Failed to change video mode: " + std::string(openni::OpenNI::getExtendedError()));
                return -3;
            }
            else
            {
                return 0;
            }
        }
    }

    for(int i=0;i < (*ppSensorInfo)->getSupportedVideoModes().getSize();i++)
    {
        openni::VideoMode videoMode = (*ppSensorInfo)->getSupportedVideoModes()[i];
        jderobot::Logger::getInstance()->info("[" + boost::lexical_cast<std::string>(i)  + "]: fps: " + boost::lexical_cast<std::string>(videoMode.getFps()) + "x: " + boost::lexical_cast<std::string>(videoMode.getResolutionX()) + "y " +  boost::lexical_cast<std::string>(videoMode.getResolutionY()));
    }


    *pbIsStreamOn = true;

    return 0;
}


void ConcurrentDevice::initConstants()
{
// 	// Primary Streams
    int nIndex = 0;

    // Registration
    nIndex = 0;



    g_Registration.pValues[nIndex++] = openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
    g_Registration.pValueToName[TRUE] = "Depth -> Image";

    g_Registration.nValuesCount = nIndex;
}


void ConcurrentDevice::readFrame()
{
    openni::Status rc = openni::STATUS_OK;

    openni::VideoStream* streams[] = {&g_depthStream, &g_colorStream, &g_irStream};

    int changedIndex = -1;
    while (rc == openni::STATUS_OK)
    {
        rc = openni::OpenNI::waitForAnyStream(streams, 3, &changedIndex, 0);
        if (rc == openni::STATUS_OK)
        {
            mutex.lock();
            switch (changedIndex)
            {
                case 0:
                    g_depthStream.readFrame(&g_depthFrame); break;
                case 1:
                    g_colorStream.readFrame(&g_colorFrame); break;
                case 2:
                    g_irStream.readFrame(&g_irFrame); break;
                default:
                    jderobot::Logger::getInstance()->error("Error in wait");

            }
            mutex.unlock();
        }
    }
}


int ConcurrentDevice::openCommon()
{
    g_pPlaybackControl = g_device.getPlaybackControl();

    int ret;

    ret = openStream( "depth", openni::SENSOR_DEPTH, config.openDepth, g_depthStream, &g_depthSensorInfo, &g_bIsDepthOn);
    if (ret != 0)
    {
        return ret;
    }

    ret = openStream( "color", openni::SENSOR_COLOR, config.openColor, g_colorStream, &g_colorSensorInfo, &g_bIsColorOn);
    if (ret != 0)
    {
        return ret;
    }

    ret = openStream( "IR", openni::SENSOR_IR, config.openIR, g_irStream, &g_irSensorInfo, &g_bIsIROn);
    if (ret != 0)
    {
        return ret;
    }

    initConstants();

    readFrame();


    return 0;
}

cv::Mat ConcurrentDevice::getDepthImage() {
    openni::DepthPixel *depthPixels;
    this->mutex.lock();
    bool validFrame=false;
    if (g_depthFrame.isValid())
    {
        validFrame=true;
        depthPixels = new openni::DepthPixel[g_depthFrame.getHeight() * g_depthFrame.getWidth()];
        memcpy(depthPixels, g_depthFrame.getData(),
               g_depthFrame.getHeight() * g_depthFrame.getWidth() * sizeof(openni::DepthPixel));
    }
    this->mutex.unlock();
    if (validFrame) {

        cv::Mat depthImage = OpenCVConverter::convertDepthToCVMat((const openni::DepthPixel *) depthPixels,
                                                                  g_depthFrame.getStrideInBytes(),
                                                                  cv::Size(g_depthFrame.getWidth(),
                                                                           g_depthFrame.getHeight()));
        free(depthPixels);
        if (this->videoMode.active){
            cv::Mat resizedImage;
            cv::resize(depthImage,resizedImage,cv::Size(this->videoMode.witdh,this->videoMode.heigth),0,0,cv::INTER_NEAREST);
            return resizedImage;
        }
        else {
            return depthImage;
        }
    }
    else{
        return cv::Mat();
    }
}

cv::Mat ConcurrentDevice::getRGBImage() {
    openni::RGB888Pixel *rgbPixels;
    bool validFrame=false;
    this->mutex.lock();
    if (g_colorFrame.isValid()) {
        validFrame=true;
        rgbPixels = new openni::RGB888Pixel[g_colorFrame.getHeight() * g_colorFrame.getWidth()];
        memcpy(rgbPixels, g_colorFrame.getData(),
               g_colorFrame.getHeight() * g_colorFrame.getWidth() * sizeof(openni::RGB888Pixel));
    }
    this->mutex.unlock();

    if (validFrame) {
        cv::Mat colorImage = OpenCVConverter::convertRGBToCVMat((const openni::RGB888Pixel *) rgbPixels,
                                                                g_colorFrame.getStrideInBytes(),
                                                                cv::Size(g_colorFrame.getWidth(),
                                                                         g_colorFrame.getHeight()));
        free(rgbPixels);
        if (this->videoMode.active){
            cv::Mat resizedImage;
            cv::resize(colorImage,resizedImage,cv::Size(this->videoMode.witdh,this->videoMode.heigth));
            return resizedImage;
        }
        else {
            return colorImage;
        }
    }
    else{
        return cv::Mat();
    }
}


void ConcurrentDevice::getDistances(std::vector<int> &distances) {
    openni::DepthPixel *depthPixels;
    this->mutex.lock();
    bool validFrame=false;
    if (g_depthFrame.isValid())
    {
        validFrame=true;
        depthPixels = new openni::DepthPixel[g_depthFrame.getHeight() * g_depthFrame.getWidth()];
        memcpy(depthPixels, g_depthFrame.getData(),
               g_depthFrame.getHeight() * g_depthFrame.getWidth() * sizeof(openni::DepthPixel));
    }
    this->mutex.unlock();
    if (validFrame) {
        distances.resize(g_depthStream.getVideoMode().getResolutionX() * g_depthStream.getVideoMode().getResolutionY());

        OpenCVConverter::convertDepthToDistances((const openni::DepthPixel *) depthPixels,
                                                                  g_depthFrame.getStrideInBytes(),
                                                                  cv::Size(g_depthFrame.getWidth(),
                                                                           g_depthFrame.getHeight()),distances);
        free(depthPixels);
    }
    else{
        distances.clear();
    }
}



void ConcurrentDevice::changeRegistration(int value)
{
    openni::ImageRegistrationMode mode = (openni::ImageRegistrationMode)value;
    if (!g_device.isValid() || !g_device.isImageRegistrationModeSupported(mode))
    {
        return;
    }

    g_device.setImageRegistrationMode(mode);
}

void ConcurrentDevice::stop() {
    this->componentAlive=false;
}

DeviceConfig ConcurrentDevice::getConfig() {
    return this->config;
}

VideoMode ConcurrentDevice::getVideoMode() {
    return this->videoMode;
}

