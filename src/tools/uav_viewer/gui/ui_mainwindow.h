/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Fri Aug 1 20:12:34 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore>
#include <QtWidgets>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QTabWidget *tabWidget;
    QWidget *view_tab;
    QGroupBox *vel_groupbox;
    QLabel *velXLabel;
    QLabel *velYLabel;
    QLabel *velZLabel;
    QGroupBox *data_groupbox;
    //QwtThermo *battery;
    QLabel *batteryLabel;
    QLabel *yawLabel;
    QLabel *altdLabel;
    QLabel *pitchLabel;
    QLabel *rollLabel;
    QLabel *pitchValue;
    QLabel *rollValue;
    QLabel *altdValue;
    QLabel *yawValue;
    QLabel *yawG;
    QLabel *pitchG;
    QLabel *rollG;
    QLabel *altdM;
    QLabel *imageLabel;
    QGroupBox *controlBox;
    QPushButton *takeoff_button;
    QPushButton *land_button;
    QPushButton *reset_button;
    QPushButton *angZL;
    QPushButton *linZD;
    QPushButton *angZR;
    QPushButton *linZU;
    QPushButton *linXF;
    QPushButton *linYR;
    QPushButton *linYL;
    QPushButton *linXB;
    QPushButton *toggleCam;
    QLabel *imageVLabel;
    QLabel *statusLabel;
    QLabel *statusValue;
    QLabel *imageLLabel;
    QCheckBox *velBox;
    QCheckBox *dataBox;
    QLabel *velLabel;
    QLabel *dataLabel;
    QWidget *gps_tab;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(900, 620);
        MainWindow->setMinimumSize(QSize(700, 620));
        MainWindow->setMaximumSize(QSize(700, 620));

        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));

        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setGeometry(QRect(10, 0, 680, 611));

        view_tab = new QWidget();
        view_tab->setObjectName(QString::fromUtf8("view_tab"));

//        battery = new QwtThermo(view_tab);
//        battery->setObjectName(QString::fromUtf8("battery"));
//        battery->setGeometry(QRect(580, 50, 56, 241));
//        battery->setMaxValue(100);
//        batteryLabel = new QLabel(view_tab);
//        batteryLabel->setObjectName(QString::fromUtf8("batteryLabel"));
//        batteryLabel->setGeometry(QRect(580, 310, 51, 21));

        velBox = new QCheckBox(view_tab);
        velBox->setObjectName(QString::fromUtf8("velBox"));
        dataBox = new QCheckBox(view_tab);
        dataBox->setObjectName(QString::fromUtf8("dataBox"));
        velBox->setGeometry(550, 370, 20, 20);
        dataBox->setGeometry(550, 400, 20, 20);
        velLabel = new QLabel(view_tab);
        velLabel->setObjectName(QString::fromUtf8("velLabel"));
        velLabel->setGeometry(QRect(570, 370, 65, 21));
        dataLabel = new QLabel(view_tab);
        dataLabel->setObjectName(QString::fromUtf8("dataLabel"));
        dataLabel->setGeometry(QRect(570, 400, 51, 21));

        imageLabel = new QLabel(view_tab);
        imageLabel->setObjectName(QString::fromUtf8("imageLabel"));
        imageLabel->setGeometry(QRect(20, 0, 320, 240));
        imageLabel->setMinimumSize(QSize(320, 240));
        imageLabel->setMaximumSize(QSize(320, 240));
        controlBox = new QGroupBox(view_tab);
        controlBox->setObjectName(QString::fromUtf8("controlBox"));
        controlBox->setGeometry(QRect(160, 380, 391, 211));
        takeoff_button = new QPushButton(controlBox);
        takeoff_button->setObjectName(QString::fromUtf8("takeoff_button"));
        takeoff_button->setGeometry(QRect(10, 40, 95, 31));
        land_button = new QPushButton(controlBox);
        land_button->setObjectName(QString::fromUtf8("land_button"));
        land_button->setGeometry(QRect(140, 40, 95, 31));
        reset_button = new QPushButton(controlBox);
        reset_button->setObjectName(QString::fromUtf8("reset_button"));
        reset_button->setGeometry(QRect(260, 40, 95, 31));
        angZL = new QPushButton(controlBox);
        angZL->setObjectName(QString::fromUtf8("angZL"));
        angZL->setGeometry(QRect(10, 140, 41, 31));
        QIcon icon;
        icon.addFile(QString::fromUtf8("../buttons/rotate_left.png"), QSize(), QIcon::Normal, QIcon::Off);
        angZL->setIcon(icon);
        linZD = new QPushButton(controlBox);
        linZD->setObjectName(QString::fromUtf8("linZD"));
        linZD->setGeometry(QRect(60, 140, 41, 31));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8("../buttons/down.png"), QSize(), QIcon::Normal, QIcon::Off);
        linZD->setIcon(icon1);
        angZR = new QPushButton(controlBox);
        angZR->setObjectName(QString::fromUtf8("angZR"));
        angZR->setGeometry(QRect(110, 140, 41, 31));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8("../buttons/rotate_right.png"), QSize(), QIcon::Normal, QIcon::Off);
        angZR->setIcon(icon2);
        linZU = new QPushButton(controlBox);
        linZU->setObjectName(QString::fromUtf8("linZU"));
        linZU->setGeometry(QRect(60, 100, 41, 31));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8("../buttons/up.png"), QSize(), QIcon::Normal, QIcon::Off);
        linZU->setIcon(icon3);
        linXF = new QPushButton(controlBox);
        linXF->setObjectName(QString::fromUtf8("linXF"));
        linXF->setGeometry(QRect(260, 100, 41, 31));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8("../buttons/forward.png"), QSize(), QIcon::Normal, QIcon::Off);
        linXF->setIcon(icon4);
        linYR = new QPushButton(controlBox);
        linYR->setObjectName(QString::fromUtf8("linYR"));
        linYR->setGeometry(QRect(310, 140, 41, 31));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8("../buttons/right.png"), QSize(), QIcon::Normal, QIcon::Off);
        linYR->setIcon(icon5);
        linYL = new QPushButton(controlBox);
        linYL->setObjectName(QString::fromUtf8("linYL"));
        linYL->setGeometry(QRect(210, 140, 41, 31));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8("../buttons/left.png"), QSize(), QIcon::Normal, QIcon::Off);
        linYL->setIcon(icon6);
        linXB = new QPushButton(controlBox);
        linXB->setObjectName(QString::fromUtf8("linXB"));
        linXB->setGeometry(QRect(260, 140, 41, 31));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8("../buttons/backward.png"), QSize(), QIcon::Normal, QIcon::Off);
        linXB->setIcon(icon7);
        toggleCam = new QPushButton(controlBox);
        toggleCam->setObjectName(QString::fromUtf8("toggleCam"));
        toggleCam->setGeometry(QRect(160, 100, 41, 31));
        QIcon icon8;
        icon8.addFile(QString::fromUtf8("../buttons/toggle_cam.png"), QSize(), QIcon::Normal, QIcon::Off);
        toggleCam->setIcon(icon8);
        imageVLabel = new QLabel(view_tab);
        imageVLabel->setObjectName(QString::fromUtf8("imageVLabel"));
        imageVLabel->setGeometry(QRect(180, 140, 200, 180));
        statusLabel = new QLabel(view_tab);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setGeometry(QRect(20, 10, 51, 21));
        statusValue = new QLabel(view_tab);
        statusValue->setObjectName(QString::fromUtf8("statusValue"));
        statusValue->setGeometry(QRect(90, 10, 65, 21));
        imageLLabel = new QLabel(view_tab);
        imageLLabel->setObjectName(QString::fromUtf8("imageLLabel"));
        imageLLabel->setGeometry(QRect(20, 40, 640, 320));
        tabWidget->addTab(view_tab, QString());

        gps_tab = new QWidget();
        gps_tab->setObjectName(QString::fromUtf8("gps_tab"));
        tabWidget->addTab(gps_tab, QString());

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "uav_viewer", 0));
        //batteryLabel->setText(QApplication::translate("MainWindow", "Battery", 0));
        velLabel->setText(QApplication::translate("MainWindow", "Velocities", 0));
        dataLabel->setText(QApplication::translate("MainWindow", "Data", 0));
        imageLabel->setText(QString());
        controlBox->setTitle(QApplication::translate("MainWindow", "Control", 0));
        takeoff_button->setText(QApplication::translate("MainWindow", "take off", 0));
        land_button->setText(QApplication::translate("MainWindow", "land", 0));
        reset_button->setText(QApplication::translate("MainWindow", "reset", 0));
        angZL->setText(QString());
        linZD->setText(QString());
        angZR->setText(QString());
        linZU->setText(QString());
        linXF->setText(QString());
        linYR->setText(QString());
        linYL->setText(QString());
        linXB->setText(QString());
        toggleCam->setText(QString());
        imageVLabel->setText(QString());
        statusLabel->setText(QApplication::translate("MainWindow", "Status:", 0));
        statusValue->setText(QString());
        imageLLabel->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(view_tab), QApplication::translate("MainWindow", "Viewer", 0));
        tabWidget->setTabText(tabWidget->indexOf(gps_tab), QApplication::translate("MainWindow", "GPS", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
