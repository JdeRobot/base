#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <easyiceconfig/EasyIce.h>
#include <jderobot/visualStates/automatagui.h>
#include <jderobot/motors.h>

typedef enum State_0 {
	state_1,
	state_2,
} State_0;

const char* State_Names_0[] = {
	"state_1",
	"state_2",
};

pthread_t thr_0;
pthread_t thr_gui;

AutomataGui* automatagui;
bool displayGui = false;

bool run0 = true;

State_0 sub_0 = state_1;

jderobot::MotorsPrx myMotors;

void shutDown() {
	run0 = false;
	automatagui->close();
}


std::list<GuiSubautomata> createGuiSubAutomataList(){
	std::list<GuiSubautomata> guiSubautomataList;

	GuiSubautomata* guiSubautomata0 = new GuiSubautomata(0, 0);

	guiSubautomata0->newGuiNode(1, 0, 857.0, 829.0);
	guiSubautomata0->setIsInitialLastGuiNode(true);
	guiSubautomata0->setNameLastGuiNode("state 1");

	guiSubautomata0->newGuiNode(2, 0, 1089.0, 1139.0);
	guiSubautomata0->setIsInitialLastGuiNode(false);
	guiSubautomata0->setNameLastGuiNode("state 2");

	Point* origin11 = new Point(857.0, 829.0);
	Point* dest11 = new Point(1089.0, 1139.0);
	Point* midPoint11 = new Point(973.0, 984.0);
	guiSubautomata0->newGuiTransition(*origin11, *dest11, *midPoint11, 1, 1, 2);
	guiSubautomataList.push_back(*guiSubautomata0);
	return guiSubautomataList;
}

void* state_0 (void*) {
	struct timeval a, b;
	int cycle = 100;
	long totala, totalb;
	long diff;
	time_t t_ini;
	time_t t_fin;
	double secs;
	bool t_activated = false;

	
	while (run0) {
		gettimeofday(&a, NULL);
		totala = a.tv_sec * 1000000 + a.tv_usec;

		//Evaluation switch
		switch (sub_0) {
			case state_1: {
				if (!t_activated) {
					t_ini = time(NULL);
					t_activated = true;
				} else {
					t_fin = time(NULL);
					secs = difftime(t_fin, t_ini);
					if (secs > (double) 10.0) {
						sub_0 = state_2;
						t_activated = false;
						
						if (displayGui) {
							automatagui->notifySetNodeAsActive("state 2");
						}
					}
				}

				break;
			}
			case state_2: {
				break;
			}
		}

		// Actuation switch
		switch (sub_0) {
			case state_1: {
				myMotors->setV(0.5);
				break;
			}
			case state_2: {
				myMotors->setV(0.0);
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
		if (diff < 33)
			usleep(33*1000);
	}
}

void* runAutomatagui(void*) {
	automatagui->run();
}

bool showAutomataGui() {
	if (automatagui->init() < 0) {
		std::cerr << "warning: could not show automatagui" << std::endl;
		return false;
	}
	automatagui->setGuiSubautomataList(createGuiSubAutomataList());
	pthread_create(&thr_gui, NULL, runAutomatagui, NULL);
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

int main(int argc, char* argv[]) {
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


        if (displayGui) {
            automatagui = new AutomataGui(argc, argv);
            displayGui = showAutomataGui();
        }

		pthread_create(&thr_0, NULL, &state_0, NULL);
		pthread_join(thr_0, NULL);

        if (displayGui)
            pthread_join(thr_gui, NULL);


    } catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
        status = 1;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
        status = 1;
    }
            
    if (ic)
        ic->destroy();
                
    return status;
}
