#ifndef LASERWIDGET_H
#define LASERWIDGET_H


#include <iostream>
#include <math.h>
#include <QtWidgets>
#include "../../robot/types.h"

class LaserWidget: public QWidget
{
public:
    LaserWidget();
    void update(LaserD laserData);

protected:
    void paintEvent(QPaintEvent *);

    QMutex mutex;
    LaserD laserData;

};

#endif // LASERWIDGET_H
