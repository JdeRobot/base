#ifndef CAMERASWIDGET_H
#define CAMERASWIDGET_H

#include <QtWidgets>
#include "parallelIce/cameraClient.h"

class CamerasWidget: public QWidget
{
    Q_OBJECT

public:
    CamerasWidget(jderobot::cameraClient* camera);

    void update();

private:
    QLabel* labelImage;
    jderobot::cameraClient* camera;
};

#endif // CAMERASWIDGET_H
