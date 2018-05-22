#ifndef CAMERASWIDGET_H
#define CAMERASWIDGET_H

#include <QtWidgets>
#include <jderobot/comm/cameraClient.hpp>
#include <jderobot/types/image.h>

class CamerasWidget: public QWidget
{
    Q_OBJECT

public:
    CamerasWidget(Comm::CameraClient* camera);

    void update();

private:
    QLabel* labelImage;
    Comm::CameraClient* camera;
};

#endif // CAMERASWIDGET_H
