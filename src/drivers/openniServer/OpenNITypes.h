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


struct VideoMode{
    int witdh;
    int heigth;
    bool valid;

    VideoMode(){
        this->valid=false;
    }

    VideoMode(int witdh, int heigth):witdh(witdh), heigth(heigth),valid(false){
    }

    void setValid(bool value){
        this->valid=value;
    }
};


#endif //OPENNISERVER_OPENNITYPES_H
