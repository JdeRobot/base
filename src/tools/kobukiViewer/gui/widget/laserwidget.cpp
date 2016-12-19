#include "laserwidget.h"

LaserWidget::LaserWidget()
{

    setAutoFillBackground(true);

    QPalette Pal(palette());
    // set black background
    Pal.setColor(QPalette::Background, Qt::black);
    setAutoFillBackground(true);
    setPalette(Pal);

    setMaximumSize(500, 500);


    setWindowTitle("Laser");
}

void LaserWidget::update(LaserD laserData)
{
    mutex.lock();

    this->laserData = laserData;
    QWidget::update();

    mutex.unlock();

}

void LaserWidget::paintEvent(QPaintEvent *)
{
    int _width = width();
    int _height = height();

    //center of laser
    int cx = _width/2;
    int cy = _height/2; 

    float x0, y0, x1, y1, d, ang;

    int width = 2;
    QPen pen;

    QPainter painter(this);
    painter.setPen(pen);

    pen = QPen(Qt::green, width);
    painter.setPen(pen);

    painter.drawPoint(QPointF(20,480));

    pen = QPen(Qt::blue, width);
    painter.setPen(pen);

    float step = (this->laserData.maxAngle - this->laserData.minAngle) /this->laserData.values.size();

    d = this->laserData.maxRange/(_width/2); //normalizing distances 

    ang = this->laserData.minAngle;
    x0 = cx + (this->laserData.values[0] / d) * cos(ang);
    y0 = cy - ((this->laserData.values[0] / d) * sin(ang));
            std::cout << "--------------------------------------------"<< std::endl;
        std::cout << "ang: " << ang*180/3.14 << std::endl;
        std::cout << "d: " << d << std::endl;
        std::cout << "x0: " << x0 << std::endl;
        std::cout << "y0: " << y0 << std::endl;
        std::cout << "--------------------------------------------"<< std::endl;
    for (int i = 1; i < this->laserData.values.size(); i++) {



        ang = this->laserData.minAngle + i*step;
        x0 = cx + (this->laserData.values[i] / d) * cos(ang);
        y0 = cy - ((this->laserData.values[i] / d) * sin(ang));
        std::cout << "--------------------------------------------"<< std::endl;
        std::cout << "ang: " << ang*180/3.14 << std::endl;
        std::cout << "d: " << d << std::endl;
        std::cout << "value: " << this->laserData.values[i] << std::endl;
        std::cout << "value/d: " << this->laserData.values[i] / d<< std::endl;
        std::cout << "x0: " << x0 << std::endl;
        std::cout << "y0: " << y0 << std::endl;
        std::cout << "--------------------------------------------"<< std::endl;

        painter.drawLine(QPointF(x0,y0), QPointF(x1,y1));

        x0 = x1;
        y0 = y1;

    }
}


