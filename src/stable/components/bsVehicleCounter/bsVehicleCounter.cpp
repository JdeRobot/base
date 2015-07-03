#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <jderobot/camera.h>
#include <jderobot/pose3d.h>
#include <jderobot/cmdvel.h>
#include <jderobot/ardroneextra.h>

#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>
#include <stdlib.h>

#define DRONE_HEIGHT 5
#define DRONE_VEL 0.5
#define EPS 0.5
#define N_FRAMES 500

int main (int argc, char** argv) {

	bool initiated = false;
	bool dynamic = false;
	int lmindex = 0;
	std::vector<cv::Point> landmarks;

	Ice::CommunicatorPtr ic;
	jderobot::ArDroneExtraPrx arextraprx;
	jderobot::Pose3DPrx poseprx;
	jderobot::CMDVelPrx cmdprx;
	jderobot::Pose3DDataPtr pose;
	jderobot::CMDVelDataPtr vel = new jderobot::CMDVelData();

	//prespecify checkpoints
	landmarks.push_back(cv::Point(5.0, 0.0));
	landmarks.push_back(cv::Point(-5.0, 0.0));

	try {
		ic = Ice::initialize(argc, argv);
		Ice::PropertiesPtr prop = ic->getProperties();

		Ice::ObjectPrx baseextra = ic->propertyToProxy("BSCounter.Extra.Proxy");
		if (0==baseextra)
			throw "Could not create proxy";
		arextraprx = jderobot::ArDroneExtraPrx::checkedCast(baseextra);

		Ice::ObjectPrx basepose = ic->propertyToProxy("BSCounter.Pose3D.Proxy");
		if (0==basepose)
			throw "Could not create proxy";
		poseprx = jderobot::Pose3DPrx::checkedCast(basepose);

		Ice::ObjectPrx basecmd = ic->propertyToProxy("BSCounter.CMDVel.Proxy");
		if (0==basecmd)
			throw "Could not create proxy";
		cmdprx = jderobot::CMDVelPrx::checkedCast(basecmd);

	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		exit(-1);
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		exit(-1);
	}

	std::cout << "taking off..\n";
	arextraprx->takeoff();

	std::cout << "reaching desired height..\n";
	vel->linearZ = DRONE_VEL;
	cmdprx->setCMDVelData(vel);

	while (1) {
		pose = poseprx->getPose3DData();
		if (!initiated && abs(pose->z - DRONE_HEIGHT) < EPS) {
			vel->linearZ = 0;
			cmdprx->setCMDVelData(vel);
			initiated = true;
			std::cout<< "reached height: "<<pose->z<<" m.\n";
			std::cout<< "[STATUS] initialized: moving to 1st checkpoint..\n";
		}

		if (initiated && !dynamic) {
			//1. set velocity for next checkpoint
			vel->linearX = (landmarks[lmindex].x-pose->x)/sqrt((landmarks[lmindex].x-pose->x)*(landmarks[lmindex].x-pose->x)+(landmarks[lmindex].y-pose->y)*(landmarks[lmindex].y-pose->y));
			vel->linearY = (landmarks[lmindex].y-pose->y)/sqrt((landmarks[lmindex].x-pose->x)*(landmarks[lmindex].x-pose->x)+(landmarks[lmindex].y-pose->y)*(landmarks[lmindex].y-pose->y));
			//TODO- take yaw into account 
			cmdprx->setCMDVelData(vel);

			//2. set dynamic = true
			dynamic = true;
			//3. Show current heatmap
		}

		if (dynamic && abs(landmarks[lmindex].x-pose->x)<EPS && abs(landmarks[lmindex].y-pose->y)<EPS) {
			std::cout << "Reached checkpoint: "<<lmindex+1<<".\n";
			std::cout << "Current position: [X="<<pose->x<<"m, Y="<<pose->y<<"m, Z="<<pose->z<<"m]\n";
			std::cout << "[STATUS] Processing started..\n";
			//1. set dynamic = false
			dynamic = false;
			//2. process Image for N Frames
			//3. Update lmindex
			++lmindex%=landmarks.size();
			std::cout << "[STATUS] Processed "<<N_FRAMES<<" frames. Moving to next checkpoint.\n";
		}
	}

	return 0;
}
