#ifndef VELWIDGET_H
#define VELWIDGET_H

#include <QtGui>
#include <qwt_compass.h>
#include <qwt_analog_clock.h>
#include "speedometer.h"
#include "../sensors/sensors.h"


class VelWidget: public QWidget
{
public:
    VelWidget(QWidget *parent = 0);
    void update();
    void setUI();
    QwtDial *createDial(int pos);
    void drawVelocitiesValues(float vlx,float vly,float vlz);
    void setSensors(Sensors* sensors);
    ~VelWidget();



protected:

    QMutex mutex;

private:
    QGridLayout *mainLayout;
    QLabel *velXLabel;
    QLabel *velYLabel;
    QLabel *velZLabel;

    QwtDial *velLinZ;
    QwtDial *velLinY;
    QwtDial *velLinX;
    SpeedoMeter *d_speedo;
    Sensors *sensors;

    float vx,vy,vz;

};

#endif // VELWIDGET_H
