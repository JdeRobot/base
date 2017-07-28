#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(updateGUI_recieved()));
	ui->setupUi(this);
//	ui->battery->setValue(60.0);

    vel = new VelWidget();;

    dataw = new DataWidget();

	linx=liny=linz=0.0;
    roll=pitch=yaw=0.0;

	ui->tabWidget->setCurrentIndex(0);

    this->initButtons();
}

void MainWindow::setSpeeds(Ice::CommunicatorPtr ic){

 	Ice::PropertiesPtr prop = ic->getProperties();

	this->max_x = std::atof( prop->getPropertyWithDefault("UAVViewer.Xmax", "3").c_str() )/10;
	this->max_y = std::atof( prop->getPropertyWithDefault("UAVViewer.Ymax", "3").c_str() )/10;
	this->max_z = std::atof( prop->getPropertyWithDefault("UAVViewer.Zmax", "3").c_str() )/10;
	this->max_yaw = std::atof( prop->getPropertyWithDefault("UAVViewer.Yawmax", "3").c_str() )/10;
}

void MainWindow::initButtons(){
    QPixmap rotateLeft(":images/rotate_left.png");
    QIcon rotateLeftIcon(rotateLeft);
    this->ui->angZL->setIcon(rotateLeftIcon);

    QPixmap rotateRight(":images/rotate_right.png");
    QIcon rotateRightIcon(rotateRight);
    this->ui->angZR->setIcon(rotateRightIcon);

    QPixmap up(":images/up.png");
    QIcon upIcon(up);
    this->ui->linZU->setIcon(upIcon);

    QPixmap down(":images/down.png");
    QIcon downIcon(down);
    this->ui->linZD->setIcon(downIcon);

    QPixmap toggle(":images/toggle_cam.png");
    QIcon toggleIcon(toggle);
    this->ui->toggleCam->setIcon(toggleIcon);

    QPixmap forward(":images/forward.png");
    QIcon forwardIcon(forward);
    this->ui->linXF->setIcon(forwardIcon);

    QPixmap backward(":images/backward.png");
    QIcon backwardIcon(backward);
    this->ui->linXB->setIcon(backwardIcon);

    QPixmap left(":images/left.png");
    QIcon leftIcon(left);
    this->ui->linYL->setIcon(leftIcon);

    QPixmap right(":images/right.png");
    QIcon rightIcon(right);
    this->ui->linYR->setIcon(rightIcon);
}


MainWindow::~MainWindow()
{
	delete sensors;
    delete vel;
    delete dataw;
	delete ui;

}

void MainWindow::updateThreadGUI()
{
	emit signal_updateGUI();
}

void MainWindow::updateGUI_recieved()
{
	cv::Mat image = this->sensors->getImage();

	QImage imageQt = QImage((const unsigned char*)(image.data),
		    image.cols,
		    image.rows,
		    image.step,
		    QImage::Format_RGB888);
		    
		    
	//int camera=0;
    	//640x360 ArDrone2
    	//320x240 ArDrone1 frontal
    	//174x144 ArDrone 1 ventral
    int x=0;
    int y=0;
    if(image.cols==IMAGE_COLS_MAX){
        x=20;
    }else{
        x=(IMAGE_COLS_MAX+20)/2-(image.cols/2);
    }
    if(image.rows==IMAGE_ROWS_MAX){
        y=40;
    }else{
        y=(IMAGE_ROWS_MAX+40)/2-(image.rows/2);
    }

    //actualizar velocidades
    if(this->ui->velBox->isChecked()){
        vel->show();
        vel->update();
    }else
        vel->hide();

    //actualizar datos nav
    if(this->ui->dataBox->isChecked()){
        dataw->show();
        dataw->update();
    }else
        dataw->hide();


	QSize size(image.cols,image.rows);
	ui->imageLLabel->move(x,y);
	ui->imageLLabel->resize(size);

	ui->imageLLabel->setHidden(false);
	ui->imageLLabel->setPixmap(QPixmap::fromImage(imageQt));

    jderobot::NavdataDataPtr navdata=this->sensors->getNavdata();
    //int batteryPer=navdata->batteryPercent;
    //ui->battery->setValue(batteryPer);
	this->setDroneStatus(navdata->state);

}

void MainWindow::printNavdata(jderobot::NavdataDataPtr data)
{
	std::cout << "-----------------Navdata---------------" << std::endl;
	std::cout << "Vehicle " << data->vehicle << std::endl;
	std::cout << "State:\t"<< data->state << "\tTimestamp:\t"<< (data->tm/1000000) << "(s)"<< std::endl;		
	std::cout << "Battery:\t"<< data->batteryPercent << "\tAltitude:\t"<< data->altd << std::endl;

	std::cout << "RotX:\t"<< data->rotX <<std::endl;
	std::cout << "RotY:\t"<< data->rotY << std::endl;
	std::cout << "RotZ:\t"<< data->rotZ << "\n" << std::endl;		

    std::cout << "Vx:\t"<< data->vx <<std::endl;
	std::cout << "Vy:\t"<< data->vy << std::endl;
	std::cout << "Vz:\t"<< data->vz << "\n" << std::endl;	
	std::cout << "Ax:\t"<< data->ax <<std::endl;	
	std::cout << "Ay:\t"<< data->ay << std::endl;
	std::cout << "Az:\t"<< data->az << "\n" << std::endl;	
	std::cout << "MagX:\t"<< data->magX << std::endl;	
	std::cout << "MagY:\t"<< data->magY << std::endl;
	std::cout << "MagZ:\t"<< data->magZ << "\n" << std::endl;
	std::cout << "Pressure:\t"<< data->pressure << "\tTemp:\t"<< data->temp << "\n" << std::endl;	

	std::cout << "WindSpeed:\t"<< data->windSpeed << std::endl;	
	std::cout << "WindAngle:\t"<< data->windAngle << std::endl;
	std::cout << "WindCompAngle:\t"<< data->windCompAngle << "\n" << std::endl;
	std::cout << "\t--Tag Detection--" << std::endl;
	std::cout << "\tTagCount: "<< data->tagsCount << std::endl;
	if(data->tagsCount != 0){
		for(unsigned int i = 0; i < data->tagsType.size(); i++)
		{
			std::cout << "\t"<< data->tagsType[i] << std::endl;
		}
		for(unsigned int i = 0; i < data->tagsXc.size(); i++)
		{
			std::cout << "\t"<< data->tagsXc[i] << std::endl;
		}
		for(unsigned int i = 0; i < data->tagsYc.size(); i++)
		{
			std::cout << "\t"<< data->tagsYc[i] << std::endl;
		}	
		for(unsigned int i = 0; i < data->tagsWidth.size(); i++)
		{
			std::cout << "\t"<< data->tagsWidth[i] << std::endl;
		}	
		for(unsigned int i = 0; i < data->tagsHeight.size(); i++)
		{
			std::cout << "\t"<< data->tagsHeight[i] << std::endl;
		}
		for(unsigned int i = 0; i < data->tagsOrientation.size(); i++)
		{
			std::cout << "\t"<< data->tagsOrientation[i] << std::endl;
		}
		for(unsigned int i = 0; i < data->tagsDistance.size(); i++)
		{
			std::cout << "\t"<< data->tagsDistance[i] << std::endl;
		}				
	}
	std::cout << "---------------------------------------" << std::endl;
}

void MainWindow::sendVelocitiesToUAV()
{
	this->sensors->sendVelocitiesToUAV(linx,liny,linz,roll,pitch,yaw);
}

void MainWindow::setSensors(Sensors* sensors){
	this->sensors=sensors; 
    vel->setSensors(this->sensors);
    dataw->setSensors(this->sensors);
}

void MainWindow::on_takeoff_button_clicked()
{
    qDebug() << "take off";
    this->sensors->takeOff();
}

void MainWindow::on_land_button_clicked()
{
    qDebug() << "land";
    this->sensors->land();
}

void MainWindow::on_toggleCam_pressed()
{
    qDebug() << "Toggle Cam";
	this->sensors->toggleCam();
}

void MainWindow::on_reset_button_clicked()
{
    qDebug() << "reset";
    this->sensors->reset();
}

void MainWindow::on_linZU_pressed()
{
    qDebug() << "Up pressed";
    this->linz=max_z;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linZU_released()
{
    qDebug() << "Up released";
    this->linz=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linZD_pressed()
{
    qDebug() << "Down pressed";
    this->linz=-max_z;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linZD_released()
{
    qDebug() << "Down released";
    this->linz=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_angZL_pressed()
{
    qDebug() << "Rotate Left pressed";
    this->yaw=max_yaw;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_angZL_released()
{
    qDebug() << "Rotate Left released";
    this->yaw=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_angZR_pressed()
{
    qDebug() << "Rotate Right pressed";
    this->yaw=-max_yaw;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_angZR_released()
{
    qDebug() << "Rotate Right released";
    this->yaw=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linXF_pressed()
{
    qDebug() << "Forward pressed";
    this->linx=this->max_x;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linXF_released()
{
    qDebug() << "Forward released";
    this->linx=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linXB_pressed()
{
    qDebug() << "Backward pressed";
    this->linx=-max_x;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linXB_released()
{
    qDebug() << "Backward released";
    this->linx=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linYL_pressed()
{
    qDebug() << "Left pressed";
    this->liny=max_y;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linYL_released()
{
    qDebug() << "Left released";
    this->liny=0.0;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linYR_pressed()
{
    qDebug() << "Right pressed";
    this->liny=-max_y;
    this->sendVelocitiesToUAV();
}

void MainWindow::on_linYR_released()
{
    qDebug() << "Right released";
    this->liny=0.0;
    this->sendVelocitiesToUAV();
}


void MainWindow::keyPressEvent( QKeyEvent *k )
{
    switch ( tolower(k->key()) ) {
        case 'w':
            on_linZU_pressed();
            break;
        case 'd':
            on_angZR_pressed();
            break;
        case 's':
            on_linZD_pressed();
            break;
        case 'a':
            on_angZL_pressed();
            break;
        case '2':
            on_linXB_pressed();
            break;
        case '8':
            on_linXF_pressed();
            break;
        case '6':
            on_linYR_pressed();
            break;
        case '4':
            on_linYL_pressed();
            break;
        case 'r':
            on_reset_button_clicked();
            break;
        case 't':
            on_takeoff_button_clicked();
            break;
        case 'l':
            on_land_button_clicked();
            break;
        case 'c':
        	on_toggleCam_pressed();
        	break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *k )
{
    switch ( tolower(k->key()) ) {
        case 'w':
            on_linZU_released();
            break;
        case 'd':
            on_angZR_released();
            break;
        case 's':
            on_linZD_released();
            break;
        case 'a':
            on_angZL_released();
            break;
        case '2':
            on_linXB_released();
            break;
        case '8':
            on_linXF_released();
            break;
        case '6':
            on_linYR_released();
            break;
        case '4':
            on_linYL_released();
            break;
    }
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
/*    if(index==0){

    }else{

    }
*/
}

void MainWindow::setDroneStatus(int state){
	switch(state){
		case 1:
			ui->statusValue->setText("Initied");
			break;
		case 2:
			ui->statusValue->setText("Landed");
			break;
		case 3:
			ui->statusValue->setText("Flying");
			break;
		case 4:
			ui->statusValue->setText("Hovering");
			break;
		case 5:
			ui->statusValue->setText("Test");
			break;
		case 6:
			ui->statusValue->setText("Tacking off");
			break;
		case 7:
			ui->statusValue->setText("Flying");
			break;
		case 8:
			ui->statusValue->setText("Landing");
			break;
		case 9:
			ui->statusValue->setText("Looping");
			break;	
		default:								
			ui->statusValue->setText("Unknown");																	
	}

}




