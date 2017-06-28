#include "sensors.h"

Sensors::Sensors(Ice::CommunicatorPtr ic)
{
	this-> ic = ic;
	Ice::PropertiesPtr prop = ic->getProperties();
	//Camera
    jderobot::ImageDataPtr data;
	Ice::ObjectPrx basecamera = ic->propertyToProxy("UAVViewer.Camera.Proxy");
    if (0==basecamera) {
        cameraON = false;
        image.create(400, 400, CV_8UC3);
        std::cout << "Camera configuration not specified" <<std::endl;
        //throw "Could not create proxy";

    }else{
        try{
            /*cast to CameraPrx*/
            cprx = jderobot::CameraPrx::checkedCast(basecamera);
            if (0==cprx)
                throw "Camera -> Invalid proxy";

            cameraON=true;
            data = cprx->getImageData("RGB8");
        }
        catch(Ice::ConnectionRefusedException& e){
            cameraON=false;
            std::cout << "Camera inactive" << std::endl;

            //create an empty image if no camera connected (avoid seg. fault)
            image.create(400, 400, CV_8UC3);
        }
    }
	
	//Navdata
	Ice::ObjectPrx basenavdata = ic->propertyToProxy("UAVViewer.Navdata.Proxy");
    if (0==basenavdata){
        navDataON = false;
        std::cout << "NavData configuration not specified" <<std::endl;
        //throw "Could not create proxy";
    }else{
        try{
            /*cast to NavdataPrx*/
            navprx = jderobot::NavdataPrx::checkedCast(basenavdata);
            if (0==navprx)
                throw "Navdata -> Invalid proxy";

            navDataON = true;
        }catch(Ice::ConnectionRefusedException& e){
            navDataON = false;
            std::cout << "NavData inactive" << std::endl;
        }
    }

	//CMDVel
	Ice::ObjectPrx basecmd = ic->propertyToProxy("UAVViewer.CMDVel.Proxy");
    if (0==basecmd) {
        cmdVelON = false;
        std::cout << "CMDVel configuration not specified" <<std::endl;
        //throw "Could not create proxy";
    }else{
        try{
            /*cast to CMDVelPrx*/
            cmdprx = jderobot::CMDVelPrx::checkedCast(basecmd);
            std::string temp;

            if (0==cmdprx)
                throw "CMDVel -> Invalid proxy";

            cmdVelON = true;
        }catch(Ice::ConnectionRefusedException& e){
            cmdVelON = false;
            std::cout << "CMDVel inactive" << std::endl;
        }
    }

	//Pose3D
	Ice::ObjectPrx basepose = ic->propertyToProxy("UAVViewer.Pose3D.Proxy");
    if (0==basepose){
        pose3dON = false;
        std::cout << "Pose3D configuration not specified" <<std::endl;
        //throw "Could not create proxy";
    }else{
        try{
            /*cast to Pose3DPrx*/
            poseprx = jderobot::Pose3DPrx::checkedCast(basepose);
            if (0==poseprx)
                throw "Pose3D -> Invalid proxy";

            pose3dON = true;
        }catch(Ice::ConnectionRefusedException& e) {
            pose3dON = false;
            std::cout << "Pose3D inactive" << std::endl;
        }
    }

    //Extra
    Ice::ObjectPrx baseextra = ic->propertyToProxy("UAVViewer.Extra.Proxy");
    if (0==baseextra){
        extraON = false;
        std::cout << "Extra configuration not specified" <<std::endl;
        //throw "Could not create proxy";
    }else{
        try{
            /*cast to ArDronePrx*/
            arextraprx = jderobot::ArDroneExtraPrx::checkedCast(baseextra);
            if (0==arextraprx)
                throw "ArDroneExtra -> Invalid proxy";

            extraON = true;
        }catch(Ice::ConnectionRefusedException& e){
            extraON = false;
            std::cout << "Pose3D inactive" << std::endl;
        }
    }

	this->tracking=false;
	this->flying=false;	
	this->rst=false;
		
}

Sensors::~Sensors(){

}

void Sensors::sendVelocitiesToUAV(float vx,float vy,float vz,float roll,float pitch,float yaw)
{
	mutexDrone.lock();
        jderobot::CMDVelDataPtr vel=new jderobot::CMDVelData();

        vel->linearX=vx;
        vel->linearY=vy;
        vel->linearZ=vz;
        vel->angularZ=yaw;
        vel->angularX=roll;
        vel->angularY=pitch;
		
        if(cmdVelON)
            cmdprx->setCMDVelData(vel);

    mutexDrone.unlock();    
}
void Sensors::update()
{
	mutex.lock();
    if(cameraON) {
        jderobot::ImageDataPtr data = cprx->getImageData("RGB8");
		image.create(data->description->height, data->description->width, CV_8UC3); 
		memcpy((unsigned char *) image.data ,&(data->pixelData[0]), image.cols*image.rows * 3);  
    }else{

        //create empty image if the interface is not connected
        image.create(400, 400, CV_8UC3);
    }
	mutex.unlock();
	mutexDrone.lock();
        if(navDataON)
            navdata=navprx->getNavdata();
        else{

            //creating empty navdata if interface not connected
            jderobot::NavdataDataPtr emptyNav = new jderobot::NavdataData();
            emptyNav->vx = 0;
            emptyNav->vy = 0;
            emptyNav->vz = 0;
            emptyNav->ax = 0;
            emptyNav->ay = 0;
            emptyNav->az = 0;
            emptyNav->rotX = 0;
            emptyNav->rotY = 0;
            emptyNav->rotZ = 0;
            emptyNav->batteryPercent = 0.0;
            emptyNav->altd = 0;
            emptyNav->vehicle = 0;
            emptyNav->state = 0;
            emptyNav->magX = 0;
            emptyNav->magY = 0;
            emptyNav->magZ = 0;
            emptyNav->pressure = 0;
            emptyNav->windAngle = 0.0;
            emptyNav->windCompAngle = 0.0;
            emptyNav->windSpeed = 0.0;
            emptyNav->tagsCount = 0;

            navdata = emptyNav;
        }
        if(pose3dON)
            pose3DDataPtr = poseprx->getPose3DData();
        else{

            //create empty pose3d if interface not connected.
            jderobot::Pose3DDataPtr emptyPose = new jderobot::Pose3DData();
            emptyPose->h = 0.0;
            emptyPose->x = 0.0;
            emptyPose->y = 0.0;
            emptyPose->z = 0.0;
            emptyPose->q0 = 0.0;
            emptyPose->q1 = 0.0;
            emptyPose->q2 = 0.0;
            emptyPose->q3 = 0.0;

            pose3DDataPtr = emptyPose;
        }
	mutexDrone.unlock();

}

cv::Mat Sensors::getImage()
{
    mutex.lock();
		cv::Mat result = image.clone();
    mutex.unlock();
    if(result.cols>IMAGE_COLS_MAX)
    {
	    if(result.rows>IMAGE_ROWS_MAX)
	    {
	    	cv::Size size(IMAGE_COLS_MAX,IMAGE_ROWS_MAX);
	    	cv::resize(result,result,size);
	    }else{
	    	cv::Size size(IMAGE_COLS_MAX,result.cols);
	    	cv::resize(result,result,size);	    
	    }    	
    }else if(result.rows>IMAGE_ROWS_MAX)
    {
    	cv::Size size(result.cols,IMAGE_ROWS_MAX);
    	cv::resize(result,result,size);
    }
    return result;
}


void Sensors::takeOff(){
	mutexDrone.lock();
    if(extraON){
		if(!rst){
            arextraprx->takeoff();
			this->flying=true;
			std::cout << "takeoff"<<std::endl;	
		}
    }
   	mutexDrone.unlock();
}

void Sensors::land(){
	mutexDrone.lock();

		if(!rst){
            if (cmdVelON) {
                jderobot::CMDVelDataPtr cmd=new jderobot::CMDVelData();
                cmd->linearX=cmd->linearY=cmd->linearZ=0;
                cmd->angularX=cmd->angularY=cmd->angularZ=0;
                cmdprx->setCMDVelData(cmd);
            }
            if (extraON) {
                arextraprx->land();
                this->flying=false;
                this->tracking=false;
            }
			std::cout << "land"<<std::endl;	
		}
   	mutexDrone.unlock();	
}

void Sensors::reset(){
	mutexDrone.lock();
    if(extraON){
        arextraprx->reset();
		this->flying=false;
		this->tracking=false;
		this->rst=!this->rst;
    }
   	mutexDrone.unlock();	
}

void Sensors::toggleCam(){
	mutex.lock();
    if(extraON)
        arextraprx->toggleCam();
	mutex.unlock();
}

jderobot::NavdataDataPtr Sensors::getNavdata(){
	jderobot::NavdataDataPtr tmp;
	mutexDrone.lock();
		tmp=navdata;
	mutexDrone.unlock();
    return tmp;
}

jderobot::Pose3DDataPtr Sensors::getPose3DData(){
	jderobot::Pose3DDataPtr tmp = new jderobot::Pose3DData();
	mutexDrone.lock();
		tmp->x=pose3DDataPtr->x;
		tmp->y=pose3DDataPtr->y;
		tmp->z=pose3DDataPtr->z;
		tmp->h=pose3DDataPtr->h;
		tmp->q0=pose3DDataPtr->q0;
		tmp->q1=pose3DDataPtr->q1;
		tmp->q2=pose3DDataPtr->q2;
		tmp->q3=pose3DDataPtr->q3;
	mutexDrone.unlock();
    return tmp;
}

