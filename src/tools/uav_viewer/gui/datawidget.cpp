#include "datawidget.h"

DataWidget::DataWidget(QWidget *parent) :
    QWidget(parent)
{
    setWindowTitle("Data");
    this->setFixedSize(500,500);
    setUI();

    pitch=roll=yaw=0.0;
}

void DataWidget::update()
{
    jderobot::NavdataDataPtr navdata=this->sensors->getNavdata();
    //this->printNavdata(navdata);
    jderobot::Pose3DDataPtr pose3DDataPtr = this->sensors->getPose3DData();

    float qw = pose3DDataPtr->q0;
    float qx = pose3DDataPtr->q1;
    float qy = pose3DDataPtr->q2;
    float qz = pose3DDataPtr->q3;
    float yawd=this->rad2deg(quatToYaw(qw, qx, qy, qz));
    float pitchd=this->rad2deg(quatToPitch(qw, qx, qy, qz));
    float rolld=this->rad2deg(quatToRoll(qw, qx, qy, qz));
    double altd=pose3DDataPtr->z;

    drawYawValues(yawd);
    drawPitchRollValues(pitchd,rolld);
    drawAltd(altd);
}

float DataWidget::rad2deg(float r)
{
    float degree=(r * 180) / M_PI;
    return degree;
}

float DataWidget::quatToRoll(float qw, float qx, float qy, float qz)
{

    double rotateXa0 = 2.0*(qy*qz + qw*qx);
    double rotateXa1 = qw*qw - qx*qx - qy*qy + qz*qz;
    float rotateX = 0.0;
    if (rotateXa0 != 0.0 && rotateXa1 != 0.0)
        rotateX = atan2(rotateXa0, rotateXa1);

    return rotateX;
}

float DataWidget::quatToPitch(float qw, float qx, float qy, float qz)
{

    double rotateYa0 = -2.0*(qx*qz - qw*qy);
    float rotateY = 0.0;
    if( rotateYa0 >= 1.0 )
        rotateY = M_PI/2.0;
    else if( rotateYa0 <= -1.0 )
        rotateY = -M_PI/2.0;
    else rotateY = asin(rotateYa0);

    return rotateY;
}

float DataWidget::quatToYaw(float qw, float qx, float qy, float qz)
{

    double rotateZa0 = 2.0*(qx*qy + qw*qz);
    double rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz;
    float rotateZ = 0.0;
    if (rotateZa0 != 0.0 && rotateZa1 != 0.0)
        rotateZ = atan2(rotateZa0, rotateZa1);

    return rotateZ;
}

void DataWidget::setUI()
{

    mainLayout = new QGridLayout();
    horizonLayout = new QVBoxLayout();
    horizonData = new QGridLayout();
    compassLayout = new QVBoxLayout();
    compassData = new QGridLayout();
    altLayout = new QVBoxLayout();
    altData = new QGridLayout();
    turnLayout = new QVBoxLayout();
    turnData = new QGridLayout();

    yawLabel = new QLabel();
    yawLabel->setObjectName(QString::fromUtf8("yawLabel"));
    //yawLabel->setGeometry(QRect(260, 130, 41, 21));

    altdLabel = new QLabel();
    altdLabel->setObjectName(QString::fromUtf8("altdLabel"));
    altdLabel->setGeometry(QRect(260, 290, 41, 21));

    pitchLabel = new QLabel();
    pitchLabel->setObjectName(QString::fromUtf8("pitchLabel"));
    //pitchLabel->setGeometry(QRect(70, 250, 41, 21));

    rollLabel = new QLabel();
    rollLabel->setObjectName(QString::fromUtf8("rollLabel"));
    rollLabel->setGeometry(QRect(70, 270, 41, 21));

    pitchValue = new QLabel();
    pitchValue->setObjectName(QString::fromUtf8("pitchValue"));
    //pitchValue->setGeometry(QRect(110, 250, 65, 21));
    pitchValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    rollValue = new QLabel();
    rollValue->setObjectName(QString::fromUtf8("rollValue"));
    rollValue->setGeometry(QRect(110, 270, 65, 21));
    rollValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    altdValue = new QLabel();
    altdValue->setObjectName(QString::fromUtf8("altdValue"));
    altdValue->setGeometry(QRect(300, 290, 65, 21));
    altdValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    yawValue = new QLabel();
    yawValue->setObjectName(QString::fromUtf8("yawValue"));
    //yawValue->setGeometry(QRect(300, 130, 65, 21));
    yawValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

    yawG = new QLabel();
    yawG->setObjectName(QString::fromUtf8("yawG"));
    //yawG->setGeometry(QRect(370, 130, 16, 21));

    pitchG = new QLabel();
    pitchG->setObjectName(QString::fromUtf8("pitchG"));
    //pitchG->setGeometry(QRect(180, 250, 16, 21));

    rollG = new QLabel();
    rollG->setObjectName(QString::fromUtf8("rollG"));
    rollG->setGeometry(QRect(180, 270, 16, 21));

    altdM = new QLabel();
    altdM->setObjectName(QString::fromUtf8("altdM"));
    altdM->setGeometry(QRect(370, 290, 16, 21));

    yawLabel->setText(QApplication::translate("DataWidget", "Yaw:", 0));
    altdLabel->setText(QApplication::translate("DataWidget", "Altd.:", 0));
    pitchLabel->setText(QApplication::translate("DataWidget", "Pitch:", 0));
    rollLabel->setText(QApplication::translate("DataWidget", "Roll:", 0));
    pitchValue->setText(QApplication::translate("DataWidget", "0", 0));
    rollValue->setText(QApplication::translate("DataWidget", "0", 0));
    altdValue->setText(QApplication::translate("DataWidget", "10", 0));
    yawValue->setText(QApplication::translate("DataWidget", "0", 0));
    yawG->setText(QApplication::translate("DataWidget", "\302\272", 0));
    pitchG->setText(QApplication::translate("DataWidget", "\302\272", 0));
    rollG->setText(QApplication::translate("DataWidget", "\302\272", 0));
    altdM->setText(QApplication::translate("DataWidget", "m", 0));

    //to get the labels together
    QSpacerItem* hSpacer = new QSpacerItem(100, 30, QSizePolicy::Ignored, QSizePolicy::Ignored);

    horizonData->addItem(hSpacer,0,0,1,1, Qt::AlignLeft);
    horizonData->addWidget(pitchLabel,0,1,Qt::AlignCenter);
    horizonData->addWidget(pitchValue,0,2,Qt::AlignCenter);
    horizonData->addWidget(pitchG,0,3,Qt::AlignCenter);
    horizonData->addItem(hSpacer,0,4,1,1, Qt::AlignRight);

    compassData->addItem(hSpacer,0,0,1,1, Qt::AlignLeft);
    compassData->addWidget(yawLabel,0,1,Qt::AlignCenter);
    compassData->addWidget(yawValue,0,2,Qt::AlignCenter);
    compassData->addWidget(yawG,0,3,Qt::AlignCenter);
    compassData->addItem(hSpacer,0,4,1,1, Qt::AlignRight);

    altData->addItem(hSpacer,0,0,1,1, Qt::AlignLeft);
    altData->addWidget(altdLabel,0,1,Qt::AlignCenter);
    altData->addWidget(altdValue,0,2,Qt::AlignCenter);
    altData->addWidget(altdM,0,3,Qt::AlignCenter);
    altData->addItem(hSpacer,0,4,1,1, Qt::AlignLeft);

    turnData->addItem(hSpacer,0,0,1,1, Qt::AlignLeft);
    turnData->addWidget(rollLabel,0,1,Qt::AlignCenter);
    turnData->addWidget(rollValue,0,2,Qt::AlignCenter);
    turnData->addWidget(rollG,0,3,Qt::AlignCenter);
    turnData->addItem(hSpacer,0,4,1,1, Qt::AlignLeft);

    horizon=new qfi_ADI();
    horizon->setGeometry(QRect(750,80,200,200));
    horizon->setMinimumSize(200,200);
    horizonLayout->addWidget(horizon);
    horizonLayout->addLayout(horizonData);

    compass=new qfi_HSI();
    compass->setGeometry(QRect(0,0,200,200));
    compass->setMinimumSize(200,200);
    //compass->setReadOnly(true);
    compassLayout->addWidget(compass);
    compassLayout->addLayout(compassData);

    altd=new qfi_ALT();
    altd->setGeometry(QRect(970,200,115,135));
    altd->setMinimumSize(115,135);
    altLayout->addWidget(altd);
    altLayout->addLayout(altData);

    turn=new qfi_TC();
    turn->setGeometry(QRect(1100,11000,200,200));
    turn->setMinimumSize(200,200);
    turnLayout->addWidget(turn);
    turnLayout->addLayout(turnData);

    mainLayout->addLayout(horizonLayout,0,0,Qt::AlignCenter);
    mainLayout->addLayout(compassLayout,0,1,Qt::AlignCenter);
    mainLayout->addLayout(altLayout,1,0,Qt::AlignCenter);
    mainLayout->addLayout(turnLayout,1,1,Qt::AlignCenter);
    setLayout(mainLayout);

}

void DataWidget::setSensors(Sensors* sensors){
    this->sensors=sensors;
}


void DataWidget::drawYawValues(float degrees)
{

    QString value=QString::number(degrees,'f',2);
    yawValue->setText(QString(value));
    compass->setHeading(degrees);
    compass->update();
}

void DataWidget::drawAltd(float meters)
{
    QString value=QString::number(meters,'f',2);
    altdValue->setText(QString(value));
    altd->setAltitude(meters);
    altd->update();
}

void DataWidget::drawPitchRollValues(float pitch, float roll)
{
    QString pString=QString::number(pitch,'f',2);
    pitchValue->setText(pString);

    QString rString=QString::number(roll,'f',2);
    rollValue->setText(rString);

    turn->setTurnRate(roll);
    turn->update();
    horizon->setPitch(pitch);
    horizon->update();
}
