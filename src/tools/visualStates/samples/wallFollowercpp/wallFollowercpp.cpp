#include <Ice/Ice.h>
#include <easyiceconfig/EasyIce.h>
#include <jderobot/visualHFSM/automatagui.h>

#include <jderobot/comm/laserClient.hpp>
#include <jderobot/comm/motorsClient.hpp>
#include <jderobot/comm/pose3dClient.hpp>

typedef enum State_Sub_1 {
	get_close_to_wall,
	align_wall,
} State_Sub_1;

const char* Names_Sub_1[] = {
	"get_close_to_wall",
	"align_wall",
};

typedef enum State_Sub_2 {
	turn_to_wall,
	turn_to_wall_ghost,
	go_to_wall,
	go_to_wall_ghost,
} State_Sub_2;

const char* Names_Sub_2[] = {
	"turn_to_wall",
	"turn_to_wall_ghost",
	"go_to_wall",
	"go_to_wall_ghost",
};

pthread_t thr_sub_1;
pthread_t thr_sub_2;
pthread_t thr_automatagui;

AutomataGui *automatagui;
bool displayGui = false;

bool run1 = true;
bool run2 = true;

State_Sub_1 sub_1 = get_close_to_wall;
State_Sub_2 sub_2 = turn_to_wall_ghost;

JdeRobotComm::LaserClient* myLaser;
JdeRobotComm::MotorsClient* myMotors;
JdeRobotComm::Pose3dClient* myPose;

void shutDown(){
	run1 = false;
	run2 = false;
	automatagui->close();
}

#define PI 3.14159265

float get_left_distance() {
	std::vector<float> distValues = myLaser->getLaserData().values;
	// there 180 values for every angle staring from the right of the robot
	float lastx = 0.0f;	
	float lasty = -distValues.at(distValues.size()-1);
	
	float valx = -cos(PI/4.0)*distValues.at(distValues.size()-45);
	float valy = -valx;

	return -lasty;
}

float get_wall_angle() {
	std::vector<float> distValues = myLaser->getLaserData().values;
	// there 180 values for every angle staring from the right of the robot
	float lastx = 0.0f;	
	float lasty = -distValues.at(distValues.size()-1);
	
	float valx = cos(PI/4.0)*distValues.at(distValues.size()-45);
	float valy = -valx;

	float angle = atan2(valy-lasty, valx-lastx);
	
	std::cout << "angle:" << angle << std::endl;

	return angle;
}

bool start_turning = false;
float starting_angle = 0.0f;
float wall_angle = 0.0f;

float normalize_angle(float angle) {
	if (angle > PI) {
		angle = angle - PI;
	} else if (angle < -PI) {
		angle = angle+PI;
	}
	return angle;
}

float get_turned_angle() {
	if (!start_turning) {
		return 0.0f;
	} else {
		float angle = myPose->getPose().yaw - starting_angle;
		std::cout << "turned:" << angle  << "norm_turned:" << normalize_angle(angle) << std::endl;
		return fabs(normalize_angle(angle));
	}
}

float get_wall_angle2() {
	std::vector<float> distValues = myLaser->getLaserData().values;
	// there 180 values for every angle staring from the right of the robot
	float lastx = 0.0f;	
	float lasty = -distValues.at(distValues.size()-1);
	
	float valx = cos(PI/4.0)*distValues.at(distValues.size()-45);
	float valy = -valx;

	float angle = atan2(valy-lasty, valx-lastx);
	
	std::cout << "angle:" << angle << std::endl;

	return angle;
}

float get_left_min_dist() {
	std::vector<float> distValues = myLaser->getLaserData().values;
	// there 180 values for every angle staring from the right of the robot
	float min_dist = 100000;
	for (int i = distValues.size()-1; i > distValues.size()-45; i--) {
		if (distValues.at(i) < min_dist) {
			min_dist = distValues.at(i);
		}
	}
	return min_dist;
}





std::list<GuiSubautomata> createGuiSubAutomataList(){
	std::list<GuiSubautomata> guiSubautomataList;

	GuiSubautomata* guiSubautomata1 = new GuiSubautomata(1, 0);

	guiSubautomata1->newGuiNode(1, 2, 111, 129);
	guiSubautomata1->setIsInitialLastGuiNode(1);
	guiSubautomata1->setNameLastGuiNode("get_close_to_wall");

	guiSubautomata1->newGuiNode(2, 0, 354, 123);
	guiSubautomata1->setIsInitialLastGuiNode(0);
	guiSubautomata1->setNameLastGuiNode("align_wall");

	Point* origin13 = new Point(111, 129);
	Point* destiny13 = new Point(354, 123);
	Point* midPoint13 = new Point(225, 48);
	guiSubautomata1->newGuiTransition(*origin13, *destiny13, *midPoint13, 3, 1, 2);

	guiSubautomataList.push_back(*guiSubautomata1);

	GuiSubautomata* guiSubautomata2 = new GuiSubautomata(2, 1);

	guiSubautomata2->newGuiNode(3, 0, 93, 168);
	guiSubautomata2->setIsInitialLastGuiNode(1);
	guiSubautomata2->setNameLastGuiNode("turn_to_wall");

	guiSubautomata2->newGuiNode(4, 0, 290, 159);
	guiSubautomata2->setIsInitialLastGuiNode(0);
	guiSubautomata2->setNameLastGuiNode("go_to_wall");

	Point* origin21 = new Point(93, 168);
	Point* destiny21 = new Point(290, 159);
	Point* midPoint21 = new Point(163, 82);
	guiSubautomata2->newGuiTransition(*origin21, *destiny21, *midPoint21, 1, 3, 4);

	guiSubautomataList.push_back(*guiSubautomata2);

	return guiSubautomataList;
}

void* subautomata_1 ( void* ) {
	struct timeval a, b;
	int cycle = 100;
	long totala, totalb;
	long diff;
	time_t t_ini;
	time_t t_fin;
	double secs;
	bool t_activated;


	while (run1) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		// Evaluation switch
		switch (sub_1) {
			case get_close_to_wall: {
				if (get_left_min_dist() < 1.5) {
					sub_1 = align_wall;
					if(displayGui){
						automatagui->notifySetNodeAsActive("align_wall");
					}
				}

				break;
			}
			case align_wall: {
				break;
			}
		}

		// Actuation switch
		switch (sub_1) {
			case get_close_to_wall: {
				//float left_dist = get_left_distance();
				//float turn_scale = 0.5f;
				
				//myMotors->sendV(0.2);
				//myMotors->sendW(left_dist*turn_scale);
				
				//float angle = get_wall_angle();
				//std::cout << "wall_angle:" << angle << std::endl;
				break;
			}
			case align_wall: {
				float angle = get_wall_angle();
				float turn_scale = -0.8f;
				
				float min_dist = get_left_min_dist();
				
				/*
				if (min_dist < 1.5) {
					myMotors->sendV(0.1);
				} else {
					myMotors->sendV(0.2);
				}
				*/
				float dist_error = 1.5 - min_dist;
				myMotors->sendW(angle*turn_scale-dist_error/3.0f);
				myMotors->sendV(0.2);
				
				
				break;
			}
		}

		gettimeofday(&b, NULL);
		totalb = b.tv_sec * 1000000 + b.tv_usec;
		diff = (totalb - totala) / 1000;
		if (diff < 0 || diff > cycle)
			diff = cycle;
		else
			diff = cycle - diff;

		usleep(diff * 1000);
		if (diff < 33 )
			usleep (33 * 1000);
	}
}

void* subautomata_2 ( void* ) {
	struct timeval a, b;
	int cycle = 100;
	long totala, totalb;
	long diff;
	time_t t_ini;
	time_t t_fin;
	double secs;
	bool t_activated;


	while (run2) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		if (sub_1 == get_close_to_wall) {
			if ( sub_2 == turn_to_wall_ghost || sub_2 == go_to_wall_ghost) {
				sub_2 = (State_Sub_2)(sub_2 - 1);
				t_ini = time(NULL);
			}
		// Evaluation switch
		switch (sub_2) {
			case turn_to_wall: {
				if (get_turned_angle() > wall_angle) {
					sub_2 = go_to_wall;
					if(displayGui){
						automatagui->notifySetNodeAsActive("go_to_wall");
					}
				}

				break;
			}
			case go_to_wall: {
				break;
			}
		}

		// Actuation switch
		switch (sub_2) {
			case turn_to_wall: {
				#define PI 3.14159265359
				
				if (!start_turning) {
					starting_angle = myPose->getPose().yaw;
					start_turning = true;
					wall_angle = get_wall_angle2()-(PI/6);
					wall_angle = fabs(normalize_angle(wall_angle));
					std::cout << "wall_angle:" << wall_angle << std::endl;
				} else {
					if (wall_angle < 0) {
						myMotors->sendW(-0.13);
					} else {
						myMotors->sendW(0.13);
					}
					myMotors->sendV(0.0);
				}
				break;
			}
			case go_to_wall: {
				// slowly approach to wall
				myMotors->sendV(0.2f);
				myMotors->sendW(0.0f);
				break;
			}
		}
		} else {
			switch (sub_2) {
				case turn_to_wall:
					sub_2 = (State_Sub_2)(sub_2 + 1);
					break;
				case go_to_wall:
					sub_2 = (State_Sub_2)(sub_2 + 1);
					break;
				default:
					break;
			}
		}

		gettimeofday(&b, NULL);
		totalb = b.tv_sec * 1000000 + b.tv_usec;
		diff = (totalb - totala) / 1000;
		if (diff < 0 || diff > cycle)
			diff = cycle;
		else
			diff = cycle - diff;

		usleep(diff * 1000);
		if (diff < 33 )
			usleep (33 * 1000);
	}
}

void* runAutomatagui (void*) {
	automatagui->run();
}

bool showAutomataGui () {
	if (automatagui->init() < 0){
		std::cerr << "warning: could not show automatagui" << std::endl;
		return false;
	}
	automatagui->setGuiSubautomataList(createGuiSubAutomataList());
	pthread_create(&thr_automatagui, NULL, &runAutomatagui, NULL);
	automatagui->loadGuiSubautomata();
	return true;
}

void readArgs(int *argc, char* argv[]){
	int i;
	std::string splitedArg;

	for(i = 0; i < *argc; i++){
		splitedArg = strtok(argv[i], "=");
		if (splitedArg.compare("--displaygui") == 0){
			splitedArg = strtok(NULL, "=");
			if (splitedArg.compare("true") == 0 || splitedArg.compare("True") == 0){
				displayGui = true;
				std::cout << "displayGui ENABLED" << std::endl;
			}else{
				displayGui = false;
				std::cout << "displayGui DISABLED" << std::endl;
			}
		}
		if(i == *argc -1){
			(*argc)--;
		}
	}
}

int main (int argc, char* argv[]) {
	int status;
	Ice::CommunicatorPtr ic;

	try {
		ic = EasyIce::initialize(argc, argv);
		readArgs(&argc, argv);


		// Connect to myLaser
		myLaser = JdeRobotComm::getLaserClient(ic, "automata.myLaser");
		if (myLaser == NULL) {
			throw "Could not create client with name myLaser";
		} else {
			std::cout << "myLaser connected" << std::endl;
		}

		// Connect to myMotors
		myMotors = JdeRobotComm::getMotorsClient(ic, "automata.myMotors");
		if (myMotors == NULL) {
			throw "Could not create client with name myMotors";
		} else {
			std::cout << "myMotors connected" << std::endl;
		}

		// Connect to myPose
		myPose = JdeRobotComm::getPose3dClient(ic, "automata.myPose");
		if (myPose == NULL) {
			throw "Could not create client with name myPose";
		} else {
			std::cout << "myPose connected" << std::endl;
		}

		if (displayGui){
			automatagui = new AutomataGui(argc, argv);
			displayGui = showAutomataGui();
		}

		pthread_create(&thr_sub_1, NULL, &subautomata_1, NULL);
		pthread_create(&thr_sub_2, NULL, &subautomata_2, NULL);

		pthread_join(thr_sub_1, NULL);
		pthread_join(thr_sub_2, NULL);
		if (displayGui)
			pthread_join(thr_automatagui, NULL);
	} catch ( const Ice::Exception& ex ) {
		std::cerr << ex << std::endl;
		status = 1;
	} catch ( const char* msg ) {
		std::cerr << msg << std::endl;
		status = 1;
	}

	if (ic)
		ic->destroy();

	return status;
}
