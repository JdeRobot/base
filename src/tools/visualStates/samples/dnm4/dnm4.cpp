#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <easyiceconfig/EasyIce.h>
#include <jderobot/visualStates/automatagui.h>

#include <jderobot/motors.h>

typedef enum State_Sub_1 {
	state5,
	state6,
} State_Sub_1;

const char* Names_Sub_1[] = {
	"state5",
	"state6",
};

typedef enum State_Sub_2 {
	state1,
	state1_ghost,
	state2,
	state2_ghost,
} State_Sub_2;

const char* Names_Sub_2[] = {
	"state1",
	"state1_ghost",
	"state2",
	"state2_ghost",
};

typedef enum State_Sub_4 {
	state,
	state_ghost,
	state7,
	state7_ghost,
} State_Sub_4;

const char* Names_Sub_4[] = {
	"state",
	"state_ghost",
	"state7",
	"state7_ghost",
};

pthread_t thr_sub_1;
pthread_t thr_sub_2;
pthread_t thr_sub_4;
pthread_t thr_automatagui;

AutomataGui *automatagui;
bool displayGui = false;

bool run1 = true;
bool run2 = true;
bool run4 = true;

State_Sub_1 sub_1 = state5;
State_Sub_2 sub_2 = state1_ghost;
State_Sub_4 sub_4 = state_ghost;

jderobot::MotorsPrx myMotors;

void shutDown(){
	run1 = false;
	run2 = false;
	run4 = false;
	automatagui->close();
}

int my_root_funct() {
	return 4;
}

void my_func_state5() {
	printf("this is a state5 function");
}



std::list<GuiSubautomata> createGuiSubAutomataList(){
	std::list<GuiSubautomata> guiSubautomataList;

	GuiSubautomata* guiSubautomata1 = new GuiSubautomata(1, 0);

	guiSubautomata1->newGuiNode(2, 2, 369, 344);
	guiSubautomata1->setIsInitialLastGuiNode(1);
	guiSubautomata1->setNameLastGuiNode("state5");

	guiSubautomata1->newGuiNode(7, 0, 121, 83);
	guiSubautomata1->setIsInitialLastGuiNode(0);
	guiSubautomata1->setNameLastGuiNode("state6");

	Point* origin14 = new Point(121, 83);
	Point* destiny14 = new Point(369, 344);
	Point* midPoint14 = new Point(245, 214);
	guiSubautomata1->newGuiTransition(*origin14, *destiny14, *midPoint14, 4, 7, 2);

	guiSubautomataList.push_back(*guiSubautomata1);

	GuiSubautomata* guiSubautomata2 = new GuiSubautomata(2, 1);

	guiSubautomata2->newGuiNode(3, 0, 80, 82);
	guiSubautomata2->setIsInitialLastGuiNode(1);
	guiSubautomata2->setNameLastGuiNode("state1");

	guiSubautomata2->newGuiNode(4, 4, 515, 391);
	guiSubautomata2->setIsInitialLastGuiNode(0);
	guiSubautomata2->setNameLastGuiNode("state2");

	Point* origin22 = new Point(80, 82);
	Point* destiny22 = new Point(515, 391);
	Point* midPoint22 = new Point(268, 251);
	guiSubautomata2->newGuiTransition(*origin22, *destiny22, *midPoint22, 2, 3, 4);

	guiSubautomataList.push_back(*guiSubautomata2);

	GuiSubautomata* guiSubautomata4 = new GuiSubautomata(4, 2);

	guiSubautomata4->newGuiNode(5, 0, 74, 119);
	guiSubautomata4->setIsInitialLastGuiNode(1);
	guiSubautomata4->setNameLastGuiNode("state");

	guiSubautomata4->newGuiNode(6, 0, 510, 453);
	guiSubautomata4->setIsInitialLastGuiNode(0);
	guiSubautomata4->setNameLastGuiNode("state7");

	Point* origin43 = new Point(74, 119);
	Point* destiny43 = new Point(510, 453);
	Point* midPoint43 = new Point(292, 286);
	guiSubautomata4->newGuiTransition(*origin43, *destiny43, *midPoint43, 3, 5, 6);

	guiSubautomataList.push_back(*guiSubautomata4);

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

	int state4 = 34;

	while (run1) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		// Evaluation switch
		switch (sub_1) {
			case state5: {
				break;
			}
			case state6: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) 1) {
						sub_1 = state5;
						t_activated = false;
						if (displayGui){
							automatagui->notifySetNodeAsActive("state5");
						}
					}
				}

				break;
			}
		}

		// Actuation switch
		switch (sub_1) {
			case state5: {
				int my_state5 = 5;
				break;
			}
			case state6: {
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

	float t_state1_max = 5;
	int my_state5_var = 5555;

	while (run2) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		if (sub_1 == state5) {
			if ( sub_2 == state1_ghost || sub_2 == state2_ghost) {
				sub_2 = (State_Sub_2)(sub_2 - 1);
				t_ini = time(NULL);
			}
		// Evaluation switch
		switch (sub_2) {
			case state1: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) t_state1_max) {
						sub_2 = state2;
						t_activated = false;
						int mytran = 1;
						if (displayGui){
							automatagui->notifySetNodeAsActive("state2");
						}
						t_state1_max = 5;
					}
				}

				break;
			}
			case state2: {
				break;
			}
		}

		// Actuation switch
		switch (sub_2) {
			case state1: {
				int mystate1 = 1;
				break;
			}
			case state2: {
				int my_state2 = 222;
				break;
			}
		}
		} else {
			switch (sub_2) {
				case state1:
					t_state1_max = 5 - difftime(t_fin, t_ini);
					sub_2 = (State_Sub_2)(sub_2 + 1);
					break;
				case state2:
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

void* subautomata_4 ( void* ) {
	struct timeval a, b;
	int cycle = 100;
	long totala, totalb;
	long diff;
	time_t t_ini;
	time_t t_fin;
	double secs;
	bool t_activated;


	while (run4) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		if (sub_2 == state2) {
			if ( sub_4 == state_ghost || sub_4 == state7_ghost) {
				sub_4 = (State_Sub_4)(sub_4 - 1);
				t_ini = time(NULL);
			}
		// Evaluation switch
		switch (sub_4) {
			case state: {
				if (122 == 123) {
					sub_4 = state7;
					if(displayGui){
						automatagui->notifySetNodeAsActive("state7");
					}
				}

				break;
			}
			case state7: {
				break;
			}
		}

		// Actuation switch
		switch (sub_4) {
			case state: {
				const char* my_state = "my state hello world";
				break;
			}
			case state7: {
				const char* my_state7 = "my 7th hello world";
				break;
			}
		}
		} else {
			switch (sub_4) {
				case state:
					sub_4 = (State_Sub_4)(sub_4 + 1);
					break;
				case state7:
					sub_4 = (State_Sub_4)(sub_4 + 1);
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
		pthread_create(&thr_sub_4, NULL, &subautomata_4, NULL);

		pthread_join(thr_sub_1, NULL);
		pthread_join(thr_sub_2, NULL);
		pthread_join(thr_sub_4, NULL);
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
