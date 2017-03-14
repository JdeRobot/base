#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets>
#include <QKeyEvent>
#include "../sensors/sensors.h"
#include "velwidget.h"
#include "datawidget.h"
#include <iostream>
#include <QTimer>
#include "ui_mainwindow.h"
#include <QDebug>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT
	    
	public:
		explicit MainWindow(QWidget *parent = 0);
		void setSensors(Sensors* sensors);
		void updateThreadGUI();    
		void setSpeeds(Ice::CommunicatorPtr ic);
		~MainWindow();
	    
	signals:
		void signal_updateGUI();
	    
	private slots:
		void keyPressEvent( QKeyEvent *k );

		void keyReleaseEvent( QKeyEvent *k );

		void updateGUI_recieved();

		void on_takeoff_button_clicked();

		void on_land_button_clicked();

		void on_reset_button_clicked();

		void on_toggleCam_pressed();

		void on_linZU_pressed();

		void on_linZU_released();

		void on_linZD_pressed();

		void on_linZD_released();

		void on_angZL_pressed();

		void on_angZL_released();

		void on_angZR_pressed();

		void on_angZR_released();

		void on_linXF_pressed();

		void on_linXF_released();

		void on_linXB_pressed();

		void on_linXB_released();

		void on_linYL_pressed();

		void on_linYL_released();

		void on_linYR_pressed();

		void on_linYR_released();

		void on_tabWidget_currentChanged(int index);

	private:
		void sendVelocitiesToUAV();

        void initButtons();
		void setDroneStatus(int state);    
		void printNavdata(jderobot::NavdataDataPtr data);		  
		float linx,liny,linz;
        float roll, pitch, yaw;
		float max_x, max_y, max_z, max_yaw;
		Ui::MainWindow *ui;
		Sensors* sensors;
        VelWidget *vel;
        DataWidget *dataw;
};

#endif // MAINWINDOW_H
