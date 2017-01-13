#include "sensors.h"

Sensors::Sensors(Ice::CommunicatorPtr ic)
{
    this-> ic = ic;
    Ice::PropertiesPtr prop = ic->getProperties();

    ////////////////////////////// Pose3D //////////////////////////////
    // Contact to POSE3D interface
    Ice::ObjectPrx basePose3D = ic->propertyToProxy("kobukiViewer.Pose3D.Proxy");
    if (0 == basePose3D) {
		pose3dON = false;
		std::cout << "Pose3D configuration not specified" <<std::endl;
        //throw "Could not create proxy with pose3D";
	}else{
		// Cast to pose3D
		try {
			p3dprx = jderobot::Pose3DPrx::checkedCast(basePose3D);
			if (0 == p3dprx)
				throw "Invalid proxy kobukiViewer.Pose3D.Proxy";

			pose3dON = true;
			std::cout << "Pose3D connected" << std::endl;
		}catch (Ice::ConnectionRefusedException& e){
			pose3dON=false;
			std::cout << "Pose3D inactive" << std::endl;
		}
	}


    ////////////////////////////// CAMERA1 /////////////////////////////

	this->camera1 = JdeRobotComm::getCameraClient(ic, "kobukiViewer.Camera1");

    ////////////////////////////// CAMERA2 /////////////////////////////
	this->camera2 = JdeRobotComm::getCameraClient(ic, "kobukiViewer.Camera2");

    ////////////////////////////// LASER //////////////////////////////
	// Contact to LASER interface

	this->laserClient = JdeRobotComm::getLaserClient(ic, "kobukiViewer.Laser");
}

JdeRobotTypes::Image Sensors::getImage1()
{
	JdeRobotTypes::Image img;

	if (this->camera1){
    	img = this->camera1->getImage();
    }

    return img;

}

JdeRobotTypes::Image Sensors::getImage2()
{
    JdeRobotTypes::Image img;

	if (this->camera2){
    	img = this->camera2->getImage();
    }

    return img;
}

void Sensors::update()
{
	if (pose3dON) {
    	pose3ddata = this->p3dprx->getPose3DData();
	    mutex.lock();
		robotx = pose3ddata->x;
		roboty = pose3ddata->y;

		//theta
		double magnitude,w,x,y,z,squ,sqx,sqy,sqz;
		magnitude = sqrt(this->pose3ddata->q0 * this->pose3ddata->q0 + this->pose3ddata->q1 * this->pose3ddata->q1 + this->pose3ddata->q2 * this->pose3ddata->q2 + this->pose3ddata->q3 * this->pose3ddata->q3);

		w = this->pose3ddata->q0 / magnitude;
		x = this->pose3ddata->q1 / magnitude;
		y = this->pose3ddata->q2 / magnitude;
		z = this->pose3ddata->q3 / magnitude;

		squ = w * w;
		sqx = x * x;
		sqy = y * y;
		sqz = z * z;

		double angle;

		angle = atan2( 2 * (x * y + w * z), squ + sqx - sqy - sqz) * 180.0 / M_PI;

		if(angle < 0)
		{
		    angle += 360.0;
		}

		this->robottheta = angle;

	    mutex.unlock();
	}

    
}

float Sensors::getRobotPoseX()
{

	float x;
	mutex.lock();
	if (pose3dON) 
	    x = this->robotx;
   	else
		x = 0;
	mutex.unlock();

    return x;
}

float Sensors::getRobotPoseY()
{
    float y;
	mutex.lock();
	if (pose3dON) 
	    y = this->roboty;
   	else
		y = 0;
	mutex.unlock();

    return y;
}

float Sensors::getRobotPoseTheta()
{
    float theta;
	mutex.lock();
	if (pose3dON) 
	    theta = this->robottheta;
   	else
		theta = 0;
	mutex.unlock();

    return theta;
}

JdeRobotTypes::LaserData Sensors::getLaserData()
{
	JdeRobotTypes::LaserData laser;

	if (this->laserClient){
    	laser = this->laserClient->getLaserData();
    }

    return laser;
}

