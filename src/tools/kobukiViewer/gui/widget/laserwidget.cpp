#include "laserwidget.h"
#include <iostream>
#include <math.h>
LaserWidget::LaserWidget()
{

    setAutoFillBackground(true);

    QPalette Pal(palette());
    // set black background
    Pal.setColor(QPalette::Background, Qt::black);
    setAutoFillBackground(true);
    setPalette(Pal);

    setMaximumSize(500, 250);


    setWindowTitle("Laser");
}

void LaserWidget::update(std::vector<float> laserData)
{
    mutex.lock();

    this->laserData = laserData;
    QWidget::update();

    mutex.unlock();

}

void LaserWidget::paintEvent(QPaintEvent *)
{
    int _width = width();

    float x0, y0, x1, y1, d, ang;

    int width = 2;
    QPen pen;

    QPainter painter(this);
    painter.setPen(pen);

    pen = QPen(Qt::blue, width);
    painter.setPen(pen);

    float PI = 3.1416;

    ang = 0;
    x0 = _width/2 + (this->laserData[0] / d) * cos(ang);
    y0 = _width/2 - ((this->laserData[0] / d) * sin(ang));
    d = 10000/(_width/2);
    for (int i = 1; i < this->laserData.size(); i++) {


        ang = i*PI/this->laserData.size();
        x1 = _width/2 + (this->laserData[i] / d) * cos(ang);
        y1 = _width/2 - ((this->laserData[i] / d) * sin(ang));

        painter.drawLine(QPointF(x0,y0), QPointF(x1,y1));

        x0 = x1;
        y0 = y1;

    }
}


