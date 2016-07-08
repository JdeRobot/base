#include "datawidget.h"

#if QT_VERSION < 0x040000
typedef QColorGroup Palette;
#else
typedef QPalette Palette;
#endif

DataWidget::DataWidget(QWidget *parent) :
    QWidget(parent)
{
    setWindowTitle("Data");
    this->setFixedSize(640,280);
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
    drawYawValues(yawd);
    drawPitchRollValues(-pitchd,-rolld);

    double altd=pose3DDataPtr->z;
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

    mainLayout = new QHBoxLayout();
    horizonLayout = new QVBoxLayout();
    horizonData = new QGridLayout();
    compassLayout = new QVBoxLayout();
    compassData = new QGridLayout();
    altLayout = new QVBoxLayout();
    altData = new QGridLayout();

    yawLabel = new QLabel();
    yawLabel->setObjectName(QString::fromUtf8("yawLabel"));
    yawLabel->setGeometry(QRect(260, 130, 41, 21));
    altdLabel = new QLabel();
    altdLabel->setObjectName(QString::fromUtf8("altdLabel"));
    altdLabel->setGeometry(QRect(260, 290, 41, 21));
    pitchLabel = new QLabel();
    pitchLabel->setObjectName(QString::fromUtf8("pitchLabel"));
    pitchLabel->setGeometry(QRect(70, 250, 41, 21));
    rollLabel = new QLabel();
    rollLabel->setObjectName(QString::fromUtf8("rollLabel"));
    rollLabel->setGeometry(QRect(70, 270, 41, 21));
    pitchValue = new QLabel();
    pitchValue->setObjectName(QString::fromUtf8("pitchValue"));
    pitchValue->setGeometry(QRect(110, 250, 65, 21));
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
    yawValue->setGeometry(QRect(300, 130, 65, 21));
    yawValue->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
    yawG = new QLabel();
    yawG->setObjectName(QString::fromUtf8("yawG"));
    yawG->setGeometry(QRect(370, 130, 16, 21));
    pitchG = new QLabel();
    pitchG->setObjectName(QString::fromUtf8("pitchG"));
    pitchG->setGeometry(QRect(180, 250, 16, 21));
    rollG = new QLabel();
    rollG->setObjectName(QString::fromUtf8("rollG"));
    rollG->setGeometry(QRect(180, 270, 16, 21));
    altdM = new QLabel();
    altdM->setObjectName(QString::fromUtf8("altdM"));
    altdM->setGeometry(QRect(370, 290, 16, 21));

    yawLabel->setText(QApplication::translate("DataWidget", "Yaw:", 0, QApplication::UnicodeUTF8));
    altdLabel->setText(QApplication::translate("DataWidget", "Altd.:", 0, QApplication::UnicodeUTF8));
    pitchLabel->setText(QApplication::translate("DataWidget", "Pitch:", 0, QApplication::UnicodeUTF8));
    rollLabel->setText(QApplication::translate("DataWidget", "Roll:", 0, QApplication::UnicodeUTF8));
    pitchValue->setText(QApplication::translate("DataWidget", "0", 0, QApplication::UnicodeUTF8));
    rollValue->setText(QApplication::translate("DataWidget", "0", 0, QApplication::UnicodeUTF8));
    altdValue->setText(QApplication::translate("DataWidget", "10", 0, QApplication::UnicodeUTF8));
    yawValue->setText(QApplication::translate("DataWidget", "0", 0, QApplication::UnicodeUTF8));
    yawG->setText(QApplication::translate("DataWidget", "\302\272", 0, QApplication::UnicodeUTF8));
    pitchG->setText(QApplication::translate("DataWidget", "\302\272", 0, QApplication::UnicodeUTF8));
    rollG->setText(QApplication::translate("DataWidget", "\302\272", 0, QApplication::UnicodeUTF8));
    altdM->setText(QApplication::translate("DataWidget", "m", 0, QApplication::UnicodeUTF8));

    horizonData->addWidget(pitchLabel,0,0,Qt::AlignCenter);
    horizonData->addWidget(pitchValue,0,1,Qt::AlignCenter);
    horizonData->addWidget(pitchG,0,2,Qt::AlignCenter);
    horizonData->addWidget(rollLabel,1,0,Qt::AlignCenter);
    horizonData->addWidget(rollValue,1,1,Qt::AlignCenter);
    horizonData->addWidget(rollG,1,2,Qt::AlignCenter);

    compassData->addWidget(yawLabel,0,0,Qt::AlignCenter);
    compassData->addWidget(yawValue,0,1,Qt::AlignCenter);

    altData->addWidget(altdLabel,0,0,Qt::AlignCenter);
    altData->addWidget(altdValue,0,1,Qt::AlignCenter);

    horizon=createDial(0);
    horizon->setGeometry(QRect(750,80,200,200));
    horizon->setMinimumSize(200,200);
    horizonLayout->addWidget(horizon);
    horizonLayout->addLayout(horizonData);

    compass=createCompass(1);
    compass->setGeometry(QRect(0,0,200,200));
    compass->setMinimumSize(200,200);
    compass->setReadOnly(true);
    compassLayout->addWidget(compass);
    compassLayout->addLayout(compassData);

    altd=createDial(2);
    altd->setGeometry(QRect(970,200,115,135));
    altd->setMinimumSize(115,135);
    altLayout->addWidget(altd);
    altLayout->addLayout(altData);

    mainLayout->addLayout(horizonLayout);
    mainLayout->addLayout(compassLayout);
    mainLayout->addLayout(altLayout);
    setLayout(mainLayout);

}

void DataWidget::setSensors(Sensors* sensors){
    this->sensors=sensors;
}


QwtCompass *DataWidget::createCompass(int pos)
{
    int c;

    Palette colorGroup;
    for ( c = 0; c < Palette::NColorRoles; c++ )
    colorGroup.setColor((Palette::ColorRole)c, QColor());

    #if QT_VERSION < 0x040000
        colorGroup.setColor(Palette::Base, backgroundColor().light(120));
    #else
        colorGroup.setColor(Palette::Base,
        palette().color(backgroundRole()).light(120));
    #endif
    colorGroup.setColor(Palette::Foreground,
    colorGroup.color(Palette::Base));

    QwtCompass *compass = new QwtCompass(this);
    compass->setLineWidth(4);
    compass->setFrameShadow(
    pos <= 2 ? QwtCompass::Sunken : QwtCompass::Raised);

    switch(pos)
    {
        case 0:
        {
            /*
            A windrose, with a scale indicating the main directions only
            */
            QMap<double, QString> map;
            map.insert(0.0, "N");
            map.insert(90.0, "E");
            map.insert(180.0, "S");
            map.insert(270.0, "W");

            compass->setLabelMap(map);

            QwtSimpleCompassRose *rose = new QwtSimpleCompassRose(4, 1);
            compass->setRose(rose);

            compass->setNeedle(
            new QwtCompassWindArrow(QwtCompassWindArrow::Style2));
            compass->setValue(60.0);
            break;
        }
        case 1:
        {
            /*
            A compass showing another needle
            */
            QMap<double, QString> map;
            map.insert(0.0, "");
            map.insert(90.0, "");
            map.insert(180.0, "");
            map.insert(270.0, "");

            compass->setLabelMap(map);

            compass->setScaleOptions(QwtDial::ScaleTicks | QwtDial::ScaleLabel);
            compass->setScaleTicks(0, 0, 3);

            compass->setNeedle(new QwtCompassMagnetNeedle(
            QwtCompassMagnetNeedle::TriangleStyle, Qt::white, Qt::red));
            compass->setValue(220.0);
            break;
        }
    }

    QPalette newPalette = compass->palette();
    for ( c = 0; c < Palette::NColorRoles; c++ )
    {
        if ( colorGroup.color((Palette::ColorRole)c).isValid() )
        {
            for ( int cg = 0; cg < QPalette::NColorGroups; cg++ )
            {
                newPalette.setColor(
                    (QPalette::ColorGroup)cg,
                    (Palette::ColorRole)c,
                    colorGroup.color((Palette::ColorRole)c));
            }
        }
    }

    for ( int i = 0; i < QPalette::NColorGroups; i++ )
    {
        QPalette::ColorGroup cg = (QPalette::ColorGroup)i;

        const QColor light =
            newPalette.color(cg, Palette::Base).light(170);
        const QColor dark = newPalette.color(cg, Palette::Base).dark(170);
        const QColor mid = compass->frameShadow() == QwtDial::Raised
            ? newPalette.color(cg, Palette::Base).dark(110)
            : newPalette.color(cg, Palette::Base).light(110);

        newPalette.setColor(cg, Palette::Dark, dark);
        newPalette.setColor(cg, Palette::Mid, mid);
        newPalette.setColor(cg, Palette::Light, light);
    }
    compass->setPalette(newPalette);

    return compass;
}

QwtDial *DataWidget::createDial(int pos)
{
    QwtDial *dial = NULL;
    switch(pos)
    {
        case 0:
        {
            d_ai = new AttitudeIndicator(this);
            dial = d_ai;
            break;
        }
        case 1:
        {
            d_speedo = new SpeedoMeter(this);
            d_speedo->setRange(0.0,8.0);
            d_speedo->setLabel("m/s");
            d_speedo->setOrigin(180);
            d_speedo->setScaleArc(0.0,270.0);
            d_speedo->setScale(-1, 2, 1.0);
            dial = d_speedo;
            break;
        }
        case 2:
        {
            d_speedo = new SpeedoMeter(this);
            d_speedo->setRange(0.0,10.0);
            d_speedo->setLabel("m");
            d_speedo->setOrigin(-90);
            d_speedo->setScaleArc(0.0,360.0);
            d_speedo->setScale(-1, 2, 1);
            dial = d_speedo;
            break;
        }

    }

    if ( dial )
    {
        dial->setReadOnly(true);
        dial->scaleDraw()->setPenWidth(3);
        dial->setLineWidth(4);
        dial->setFrameShadow(QwtDial::Sunken);
    }
    return dial;
}


void DataWidget::drawYawValues(float degress)
{
    //Rot Z
    //-180,180
    //- right,+ left
    //Compass 360º
    QString value=QString::number(degress,'f',2);
    yawValue->setText(QString(value));
    compass->setValue(degress);
}

void DataWidget::drawAltd(float meters)
{
    int cent=0;
    float result=0.0;
    if(meters>=10 && meters<100){
        cent=(int)meters/10;
        result=meters-(cent*10);
        altd->setValue(result);
    }else if((meters>=100 && meters<1000)){
        cent=(int)meters/100;
        result=meters-(cent*100);
        altd->setValue(result);
    }else{
        altd->setValue(meters);
    }


    QString value=QString::number(meters,'f',2);
    altdValue->setText(value);
}

void DataWidget::drawPitchRollValues(float pitch,float roll)
{
    float resultP=0.0;
    if(pitch>0 && pitch<=90){
        resultP=pitch/90;
        resultP=-resultP;
    }else if(pitch<0 && pitch>=-90){
        resultP=pitch/-90;
    }else{
        resultP=0.0;
    }

    //rotY
    d_ai->setGradient(resultP);
    //rotX
    d_ai->setAngle(roll);
    QString pString=QString::number(pitch,'f',2);
    QString rString=QString::number(roll,'f',2);

    pitchValue->setText(pString);
    rollValue->setText(rString);

}
