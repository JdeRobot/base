//
// Created by frivas on 23/01/17.
//

#ifndef OPENNISERVER_OPENNITYPES_H
#define OPENNISERVER_OPENNITYPES_H

#define MAX_STRINGS 20

typedef enum SensorOpenType
{
    SENSOR_OFF,
    SENSOR_ON,
    SENSOR_TRY
} SensorOpenType;


enum RegistrationMode{ DEPTH_COLOR_OVERLAPED, DETH_RGB_REGISTERED, NONE};


typedef struct
{
    SensorOpenType openDepth;
    SensorOpenType openColor;
    SensorOpenType openIR;
    RegistrationMode registrationMode;

} DeviceConfig;


typedef struct
{
    int nValuesCount;
    unsigned int pValues[MAX_STRINGS];
    const char* pValueToName[MAX_STRINGS];
} DeviceParameter;


struct VideoModel{
    int witdh;
    int heigth;
    bool active;

    VideoModel(){
        this->active=false;
    }

    VideoModel(int mode){
        this->active=true;
        switch (mode){
            case 0:
                witdh=320;
                heigth=240;
                break;
            case 2:
                witdh=320;
                heigth=240;
                break;
            case 4:
                witdh=640;
                heigth=480;
                break;
            case 6:
                witdh=320;
                heigth=240;
                break;
            case 8:
                witdh=640;
                heigth=480;
                break;
            default:
                this->active=false;
                std::cout << "Mode not supported" << std::endl;
                break;
        }
    }
};


#endif //OPENNISERVER_OPENNITYPES_H
