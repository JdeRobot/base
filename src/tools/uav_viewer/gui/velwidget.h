#ifndef VELWIDGET_H
#define VELWIDGET_H

#include <QtWidgets>
#include "../sensors/sensors.h"
#include <iostream>
#include <qfi_SI.h>

class VelWidget: public QWidget
{
public:
    VelWidget(QWidget *parent = 0);
    void update();
    void setUI();
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

    qfi_SI *velLinZ;
    qfi_SI *velLinY;
    qfi_SI *velLinX;

    Sensors *sensors;

    float vx,vy,vz;

};

#endif // VELWIDGET_H
