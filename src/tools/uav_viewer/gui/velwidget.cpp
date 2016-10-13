#include "velwidget.h"

VelWidget::VelWidget(QWidget *parent) :
    QWidget(parent)
{

    //setAutoFillBackground(true);

    setFixedSize(700, 300);

    vx = vy = vz = 0.0;

    setWindowTitle("Velocities");
    setUI();
}

VelWidget::~VelWidget(){}

void VelWidget::setSensors(Sensors* sensors){
    this->sensors=sensors;
}

void VelWidget::update()
{
    jderobot::NavdataDataPtr navdata = this->sensors->getNavdata();
    drawVelocitiesValues(navdata->vx,navdata->vy,navdata->vz);
}

void VelWidget::setUI()
{

    mainLayout = new QGridLayout();

    velXLabel = new QLabel();
    velXLabel->setObjectName(QString::fromUtf8("velXLabel"));
    velXLabel->setGeometry(QRect(10, 10, 51, 21));
    velYLabel = new QLabel();
    velYLabel->setObjectName(QString::fromUtf8("velYLabel"));
    velYLabel->setGeometry(QRect(220, 190, 51, 21));
    velZLabel = new QLabel();
    velZLabel->setObjectName(QString::fromUtf8("velZLabel"));
    velZLabel->setGeometry(QRect(370, 190, 51, 21));

    velXLabel->setText(QString::fromUtf8("LinealX"));
    velYLabel->setText(QString::fromUtf8("LinealY"));
    velZLabel->setText(QString::fromUtf8("LinealZ"));

//    velXLabel->setText(QApplication::translate("VelWidget", "Lineal X", 0));
//    velYLabel->setText(QApplication::translate("VelWidget", "Lineal Y", 0));
//    velZLabel->setText(QApplication::translate("VelWidget", "Lineal Z", 0));

    velLinZ = new qfi_SI();
    velLinZ->setGeometry(QRect(1050,400,200,200));


    velLinY = new qfi_SI();
    velLinY->setGeometry(QRect(900,400,200,200));


    velLinX = new qfi_SI();
    velLinX->setGeometry(QRect(750,400,200,200));


    mainLayout->addWidget(velLinX,0,0,Qt::AlignCenter);
    mainLayout->addWidget(velXLabel,1,0,Qt::AlignCenter);
    mainLayout->addWidget(velLinY,0,1,Qt::AlignCenter);
    mainLayout->addWidget(velYLabel,1,1,Qt::AlignCenter);
    mainLayout->addWidget(velLinZ,0,2,Qt::AlignCenter);
    mainLayout->addWidget(velZLabel,1,2,Qt::AlignCenter);


    setLayout(mainLayout);


}

void VelWidget::drawVelocitiesValues(float vlx,float vly,float vlz)
{
//    //mm/sec to m/s
//    float resultX = 0.0;
//    float resultY = 0.0;
//    float resultZ = 0.0;
//   /* if(vlx<0.0){
//        velXLabel->setStyleSheet("QLabel {background-color: red}");
//    }else{
//        velXLabel->setStyleSheet("QLabel {background-color: green}");
//    }*/
//    vlx=std::abs(vlx);
//    vlx=vlx/1000.0;
//    resultX=(0.6*vx)+((1-0.6)*vlx);
//    velLinX->setSpeed(resultX);
//    vx=vlx;

//    /*if(vly<0.0){
//        velYLabel->setStyleSheet("QLabel {background-color: red}");
//    }else{
//        velYLabel->setStyleSheet("QLabel {background-color: green}");
//    }*/
//    vly=std::abs(vly);
//    vly=vly/1000.0;
//    resultY=(0.6*vy)+((1-0.6)*vly);
//    velLinY->setSpeed(resultY);
//    vy=vly;

//    /*if(vlz<0.0){
//        velZLabel->setStyleSheet("QLabel {background-color: red}");
//    }else{
//        velZLabel->setStyleSheet("QLabel {background-color: green}");
//    }*/
//    vlz=std::abs(vlz);
//    vlz=vlz/1000.0;
//    resultZ=(0.6*vz)+((1-0.6)*vlz);
//    velLinZ->setSpeed(resultZ);
//    vz=vlz;

    velLinY->setSpeed(vlx);
    velLinY->setSpeed(vly);
    velLinY->setSpeed(vlz);

    velLinX->update();
    velLinY->update();
    velLinZ->update();
}
