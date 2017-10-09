#include <Ice/Ice.h>
#include <easyiceconfig/EasyIce.h>
#include <jderobot/visualHFSM/automatagui.h>

#include <jderobot/comm/laserClient.hpp>

typedef enum State_Sub_1 {
	state,
} State_Sub_1;

const char* Names_Sub_1[] = {
	"state",
};

pthread_t thr_sub_1;
pthread_t thr_automatagui;

AutomataGui *automatagui;
bool displayGui = false;

bool run1 = true;

State_Sub_1 sub_1 = state;

JdeRobotComm::LaserClient* myLaser;

void shutDown(){
	run1 = false;
	automatagui->close();
}




std::list<GuiSubautomata> createGuiSubAutomataList(){
	std::list<GuiSubautomata> guiSubautomataList;

	GuiSubautomata* guiSubautomata1 = new GuiSubautomata(1, 0);

	guiSubautomata1->newGuiNode(1, 0, 166, 183);
	guiSubautomata1->setIsInitialLastGuiNode(1);
	guiSubautomata1->setNameLastGuiNode("state");

	guiSubautomataList.push_back(*guiSubautomata1);

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
			case state: {
				break;
			}
		}

		// Actuation switch
		switch (sub_1) {
			case state: {
				std::cout << "numLaser values:" << myLaser->getLaserData().values.size() << std::endl;
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
		
		myLaser = JdeRobotComm::getLaserClient(ic, "automata.myLaser");
		
		if (myLaser == NULL)
			throw "Could not create proxy with myLaser";

		if (displayGui){
			automatagui = new AutomataGui(argc, argv);
			displayGui = showAutomataGui();
		}

		pthread_create(&thr_sub_1, NULL, &subautomata_1, NULL);

		pthread_join(thr_sub_1, NULL);
		if (displayGui)
			pthread_join(thr_automatagui, NULL);
	} catch ( const Ice::Exception& ex ) {
		std::cerr << ex << std::endl;
		status = 1;
	} catch ( const char* msg ) {
		std::cerr << msg << std::endl;
		status = 1;
	}

	if (ic) {
		ic->destroy();
	}

	return status;
}
