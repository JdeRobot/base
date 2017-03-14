#ifndef GUI_H
#define GUI_H

#include <QtWidgets>
#include "widget/controlvw.h"
#include "widget/cameraswidget.h"
#include "parallelIce/motorsClient.h"

class GUI:public QWidget
{
    Q_OBJECT

public:
    GUI(Ice::CommunicatorPtr ic, jderobot::cameraClient* camera, jderobot::motorsClient* motors);
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

    jderobot::cameraClient* camera;
    jderobot::motorsClient* motors;


signals:
    void signal_updateGUI();

public slots:
    void on_updateGUI_recieved();
    void on_buttonStopRobot_clicked();

    void on_update_canvas_recieved(float v, float w);
    void on_checks_changed();

};

#endif // GUI_H
