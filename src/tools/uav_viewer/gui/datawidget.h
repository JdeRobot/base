#ifndef DATAWIDGET_H
#define DATAWIDGET_H

#include <QtGui>
#include <qwt_compass.h>
#include <qwt_analog_clock.h>
#include "speedometer.h"
#include "../sensors/sensors.h"
#include "attitudeindicator.h"
#include <QPalette>
#include <qwt_compass_rose.h>
#include <qwt_dial_needle.h>
#include <iostream>


class DataWidget: public QWidget
{
public:
    DataWidget(QWidget *parent = 0);
    void update();
    void setUI();
    QwtCompass *createCompass(int pos);
    QwtDial *createDial(int pos);
    void setSensors(Sensors* sensors);

    void drawYawValues(float degress);
    void drawAltd(float meters);
    void drawPitchRollValues(float pitch,float roll);

    static float quatToRoll(float, float, float, float);
    static float quatToPitch(float, float, float, float);
    static float quatToYaw(float, float, float, float);
    float rad2deg(float r);



protected:

    QMutex mutex;

private:
    QHBoxLayout *mainLayout;
    QVBoxLayout *horizonLayout;
    QGridLayout *horizonData;
    QVBoxLayout *compassLayout;
    QGridLayout *compassData;
    QVBoxLayout *altLayout;
    QGridLayout *altData;

    QLabel *yawLabel;
    QLabel *altdLabel;
    QLabel *pitchLabel;
    QLabel *rollLabel;
    QLabel *pitchValue;
    QLabel *rollValue;
    QLabel *altdValue;
    QLabel *yawValue;
    QLabel *yawG;
    QLabel *pitchG;
    QLabel *rollG;
    QLabel *altdM;


    QwtCompass *compass;
    QwtDial *altd;
    QwtDial *horizon;


    QwtDial *velLinZ;
    QwtDial *velLinY;
    QwtDial *velLinX;
    SpeedoMeter *d_speedo;
    Sensors *sensors;

    AttitudeIndicator *d_ai;

    float pitch, roll, yaw;

};

#endif // DATAWIDGET_H
