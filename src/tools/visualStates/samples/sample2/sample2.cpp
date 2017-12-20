#include "sample2.h"
#include <iostream>
#include <string>
#include <runtimegui.h>

void State0::runCode() {
	
}

void State1::runCode() {
	
}

void State2::runCode() {
	
}

void Tran1::runCode() {
	
}

void Tran2::runCode() {
	
}

void Interfaces::connectProxies(int argc, char* argv[]) {
	Config::Properties props = Config::load(argc, argv);
	jdrc = new Comm::Communicator(props);

	pose3d = Comm::getPose3dClient(jdrc, "sample2.pose3d");
	if (pose3d == NULL) {
		throw "invalid proxy pose3d";
	}
	std::cout << "pose3d is connected" << std::endl;
}

void Interfaces::destroyProxies() {
	if (jdrc != 0) {
	}
}


pthread_t guiThread;    
RunTimeGui* runTimeGui = NULL;
bool displayGui = false;

void readArgs(int *argc, char* argv[]) {
	int i;
	std::string splitedArg;

	for(i = 0; i < *argc; i++) {
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

void* runGui(void*) {
	system("./sample2_runtime.py");
}

int main(int argc, char* argv[]) {
	Interfaces interfaces;
	try {
		interfaces.connectProxies(argc, argv);
	} catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		interfaces.destroyProxies();
		return 1;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		interfaces.destroyProxies();
		return 1;
	}

	readArgs(&argc, argv);

	if (displayGui) {
		pthread_create(&guiThread, NULL, &runGui, NULL);
		runTimeGui = new RunTimeGui();

	}
	State* state0 = new State0(0, true, &interfaces, 100, NULL, runTimeGui);
	State* state1 = new State1(1, true, &interfaces, 100, state0, runTimeGui);
	State* state2 = new State2(2, false, &interfaces, 100, state0, runTimeGui);

	Transition* tran1 = new Tran1(1, 2, 1000);
	state1->addTransition(tran1);
	Transition* tran2 = new Tran2(2, 1, 1000);
	state2->addTransition(tran2);

	state0->startThread();

	state0->join();
}
