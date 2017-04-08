//
// Created by frivas on 23/01/17.
//

#ifndef SAMPLERGENERATOR_OPENNIDEVICELISTENER_H
#define SAMPLERGENERATOR_OPENNIDEVICELISTENER_H


#include <OpenNI.h>
#include <iostream>

class OpenNIDeviceListener : public openni::OpenNI::DeviceStateChangedListener,
                             public openni::OpenNI::DeviceDisconnectedListener
{
public:
    OpenNIDeviceListener(openni::Device* g_device):g_device(g_device){};
    virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState errorState)
    {
        if (strcmp(pInfo->getUri(), g_device->getDeviceInfo().getUri()) == 0)
        {
            if (errorState != 0)
            {
                std::cout << "Device is in error state! (error %d)"<< errorState << std::endl;
            }
            else
            {
                std::cout << " " << std::endl;
            }
        }
    }
    virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
    {
        if (strcmp(pInfo->getUri(), g_device->getDeviceInfo().getUri()) == 0)
        {
            std::cout << "Device disconnected!" << std::endl;

        }
    }
private:
    openni::Device* g_device;
};



#endif //SAMPLERGENERATOR_OPENNIDEVICELISTENER_H
