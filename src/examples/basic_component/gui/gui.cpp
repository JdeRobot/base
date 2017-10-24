#include "gui.h"

GUI::GUI(Comm::Communicator* jdrc, Comm::CameraClient* camera, Comm::MotorsClient* motors)
{

    this->camera = camera;
    this->motors = motors;


    QGridLayout* mainLayout = new QGridLayout();
    QGridLayout* layoutControl = new QGridLayout();
    QVBoxLayout* layoutButtons = new QVBoxLayout();


    camerasWidget = new CamerasWidget(this->camera);

    buttonStopRobot = new QPushButton("Stop Robot");
    checkCameras = new QCheckBox("Camera");

	InfoCurrentV = new QLabel("Current v (m/s):");
	InfoCurrentW = new QLabel("Current w (rad/s):");
	currentV = new QLabel("0");
	currentW = new QLabel("0");

    canvasVW = new controlVW();
    canvasVW->setjdrc(jdrc);

    layoutControl->addWidget(canvasVW, 0, 0);

    layoutButtons->addWidget(InfoCurrentV, 0);
    layoutButtons->addWidget(currentV, 1);
    layoutButtons->addWidget(InfoCurrentW, 2);
    layoutButtons->addWidget(currentW, 3);

	QSpacerItem *item = new QSpacerItem(0,200, QSizePolicy::Expanding, QSizePolicy::Fixed);
	layoutButtons->addItem(item);

    layoutButtons->addWidget(buttonStopRobot, 2);
    layoutButtons->addWidget(checkCameras, 4);

    mainLayout->addLayout(layoutControl, 0, 0);
    mainLayout->addLayout(layoutButtons, 0, 1);

    setLayout(mainLayout);
    setVisible(true);
    adjustSize();

    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(on_updateGUI_recieved()));
    connect(buttonStopRobot, SIGNAL(clicked()),this, SLOT(on_buttonStopRobot_clicked()) );
    connect(canvasVW, SIGNAL(VW_changed(float,float)), this, SLOT(on_update_canvas_recieved(float, float)));
    connect(checkCameras, SIGNAL(stateChanged(int)), this, SLOT(on_checks_changed()));

    show();

}

void GUI::on_checks_changed()
{
    camerasWidget->setVisible(checkCameras->isChecked());
}

void GUI::on_update_canvas_recieved(float v, float w)
{

    JdeRobotTypes::CMDVel vel;
    vel.vx = v;
    vel.az = w;
    if (this->motors) {
        this->motors->sendVelocities(vel);
    }
}

void GUI::on_buttonStopRobot_clicked()
{
    canvasVW->Stop();
}

void GUI::updateThreadGUI()
{
    emit signal_updateGUI();
}

void GUI::on_updateGUI_recieved()
{
    camerasWidget->update();
	currentV->setText( QString::number(canvasVW->getV()));
	currentW->setText( QString::number(canvasVW->getW()));
}