#ifndef DATAWIDGET_H
#define DATAWIDGET_H

#include <QtWidgets>
#include "../sensors/sensors.h"
#include <iostream>
#include <qfi_ADI.h>
#include <qfi_ALT.h>
#include <qfi_TC.h>
#include <qfi_HSI.h>


class DataWidget: public QWidget
{
public:
    DataWidget(QWidget *parent = 0);
    void update();
    void setUI();
    void setSensors(Sensors* sensors);

    void drawYawValues(float degrees);
    void drawAltd(float meters);
    void drawPitchRollValues(float pitch,float roll);

    static float quatToRoll(float, float, float, float);
    static float quatToPitch(float, float, float, float);
    static float quatToYaw(float, float, float, float);
    float rad2deg(float r);



protected:

    QMutex mutex;

private:

    QGridLayout *mainLayout;
    QVBoxLayout *horizonLayout;
    QGridLayout *horizonData;
    QVBoxLayout *compassLayout;
    QGridLayout *compassData;
    QVBoxLayout *altLayout;
    QGridLayout *altData;
    QVBoxLayout *turnLayout;
    QGridLayout *turnData;

    // name of label
    QLabel *yawLabel;
    QLabel *altdLabel;
    QLabel *pitchLabel;
    QLabel *rollLabel;

    //value of label
    QLabel *pitchValue;
    QLabel *rollValue;
    QLabel *altdValue;
    QLabel *yawValue;

    //unit of value: m, degrees
    QLabel *yawG;
    QLabel *pitchG;
    QLabel *rollG;
    QLabel *altdM;

    //graphic instruments
    qfi_ADI *horizon;
    qfi_HSI *compass;
    qfi_TC *turn;
    qfi_ALT *altd;

    Sensors *sensors;

    float pitch, roll, yaw;

};

#endif // DATAWIDGET_H
