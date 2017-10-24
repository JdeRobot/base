#ifndef GUI_H
#define GUI_H

#include <QtWidgets>
#include "widget/controlvw.h"
#include "widget/cameraswidget.h"
#include <jderobot/comm/motorsClient.hpp>
#include <jderobot/types/cmdvel.h>

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Comm::Communicator* jdrc, Comm::CameraClient* camera, Comm::MotorsClient* motors);
    void updateThreadGUI();

private:
    QPushButton* buttonStopRobot;

    controlVW* canvasVW;
    CamerasWidget* camerasWidget;

    QCheckBox* checkCameras;
    QLabel* currentV;
    QLabel* currentW;
    QLabel* InfoCurrentV;
    QLabel* InfoCurrentW;

    Comm::CameraClient* camera;
    Comm::MotorsClient* motors;


signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();
    void on_buttonStopRobot_clicked();

    void on_update_canvas_recieved(float v, float w);
    void on_checks_changed();

};

#endif // GUI_H
