#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <easyiceconfig/EasyIce.h>
#include <jderobot/visualStates/automatagui.h>

#include <jderobot/motors.h>

typedef enum State_Sub_1 {
	state1,
	state2,
} State_Sub_1;

const char* Names_Sub_1[] = {
	"state1",
	"state2",
};

typedef enum State_Sub_2 {
	state11,
	state11_ghost,
	state12,
	state12_ghost,
} State_Sub_2;

const char* Names_Sub_2[] = {
	"state11",
	"state11_ghost",
	"state12",
	"state12_ghost",
};

pthread_t thr_sub_1;
pthread_t thr_sub_2;
pthread_t thr_automatagui;

AutomataGui *automatagui;
bool displayGui = false;

bool run1 = true;
bool run2 = true;

State_Sub_1 sub_1 = state1;
State_Sub_2 sub_2 = state11_ghost;

jderobot::MotorsPrx myMotors;

void shutDown(){
	run1 = false;
	run2 = false;
	automatagui->close();
}

int functions(int a, int b) {
	return a+b;
}



std::list<GuiSubautomata> createGuiSubAutomataList(){
	std::list<GuiSubautomata> guiSubautomataList;

	GuiSubautomata* guiSubautomata1 = new GuiSubautomata(1, 0);

	guiSubautomata1->newGuiNode(1, 2, 118, 114);
	guiSubautomata1->setIsInitialLastGuiNode(1);
	guiSubautomata1->setNameLastGuiNode("state1");

	guiSubautomata1->newGuiNode(2, 0, 354, 313);
	guiSubautomata1->setIsInitialLastGuiNode(0);
	guiSubautomata1->setNameLastGuiNode("state2");

	Point* origin11 = new Point(118, 114);
	Point* destiny11 = new Point(354, 313);
	Point* midPoint11 = new Point(299, 156);
	guiSubautomata1->newGuiTransition(*origin11, *destiny11, *midPoint11, 1, 1, 2);

	Point* origin13 = new Point(354, 313);
	Point* destiny13 = new Point(118, 114);
	Point* midPoint13 = new Point(171, 304);
	guiSubautomata1->newGuiTransition(*origin13, *destiny13, *midPoint13, 3, 2, 1);

	guiSubautomataList.push_back(*guiSubautomata1);

	GuiSubautomata* guiSubautomata2 = new GuiSubautomata(2, 1);

	guiSubautomata2->newGuiNode(3, 0, 147, 123);
	guiSubautomata2->setIsInitialLastGuiNode(1);
	guiSubautomata2->setNameLastGuiNode("state11");

	guiSubautomata2->newGuiNode(4, 0, 407, 347);
	guiSubautomata2->setIsInitialLastGuiNode(0);
	guiSubautomata2->setNameLastGuiNode("state12");

	Point* origin22 = new Point(147, 123);
	Point* destiny22 = new Point(407, 347);
	Point* midPoint22 = new Point(277, 235);
	guiSubautomata2->newGuiTransition(*origin22, *destiny22, *midPoint22, 2, 3, 4);

	guiSubautomataList.push_back(*guiSubautomata2);

	return guiSubautomataList;
}

void* subautomata_1 ( void* ) {
	struct timeval a, b;
	int cycle = 200;
	long totala, totalb;
	long diff;
	time_t t_ini;
	time_t t_fin;
	double secs;
	bool t_activated;

	int variables = 10;

	while (run1) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		// Evaluation switch
		switch (sub_1) {
			case state1: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) 3) {
						sub_1 = state2;
						t_activated = false;
						int transition_code = 12;
						if (displayGui){
							automatagui->notifySetNodeAsActive("state2");
						}
					}
				}

				break;
			}
			case state2: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) 3) {
						sub_1 = state1;
						t_activated = false;
						if (displayGui){
							automatagui->notifySetNodeAsActive("state1");
						}
					}
				}

				break;
			}
		}

		// Actuation switch
		switch (sub_1) {
			case state1: {
				int state1_code = 10;
				break;
			}
			case state2: {
				int state2_code = 10;
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

	float t_state11_max = 5;

	while (run2) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		if (sub_1 == state1) {
			if ( sub_2 == state11_ghost || sub_2 == state12_ghost) {
				sub_2 = (State_Sub_2)(sub_2 - 1);
				t_ini = time(NULL);
			}
		// Evaluation switch
		switch (sub_2) {
			case state11: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) t_state11_max) {
						sub_2 = state12;
						t_activated = false;
						if (displayGui){
							automatagui->notifySetNodeAsActive("state12");
						}
						t_state11_max = 5;
					}
				}

				break;
			}
			case state12: {
				break;
			}
		}

		// Actuation switch
		switch (sub_2) {
			case state11: {
				myMotors->setV(0.2);
				break;
			}
			case state12: {
				myMotors->setV(0.0);
				break;
			}
		}
		} else {
			switch (sub_2) {
				case state11:
					t_state11_max = 5 - difftime(t_fin, t_ini);
					sub_2 = (State_Sub_2)(sub_2 + 1);
					break;
				case state12:
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


		// Contact to myMotors
		Ice::ObjectPrx temp_myMotors = ic->propertyToProxy("automata.myMotors.Proxy");
		if (temp_myMotors == 0)
			throw "Could not create proxy with myMotors";
		myMotors = jderobot::MotorsPrx::checkedCast(temp_myMotors);
		if (myMotors == 0)
			throw "Invalid proxy automata.myMotors.Proxy";
		std::cout << "myMotors connected" << std::endl;

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
