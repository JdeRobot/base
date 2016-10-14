#ifndef LASERWIDGET_H
#define LASERWIDGET_H

#include <QtWidgets>

class LaserWidget: public QWidget
{
public:
    LaserWidget();
    void update(std::vector<float> laserData);

protected:
    void paintEvent(QPaintEvent *);

    QMutex mutex;
    std::vector<float> laserData;

};

#endif // LASERWIDGET_H
