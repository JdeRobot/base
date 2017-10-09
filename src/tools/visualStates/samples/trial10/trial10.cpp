#include "trial10.h"
#include <iostream>
#include <string>
#include <runtimegui.h>

void State0::runCode() {
	
}

void State1::runCode() {
	
}

void State2::runCode() {
	interfaces->myMotors->setV(0.0);
}

void State3::runCode() {
	interfaces->myMotors->setV(0.2);
}

void State4::runCode() {
	interfaces->myMotors->setV(0.0);
}

void Tran1::runCode() {
	
}

void Tran5::runCode() {
	
}

void Tran2::runCode() {
	
}

void Tran4::runCode() {
	
}

void Interfaces::connectProxies(int argc, char* argv[]) {
	ice = EasyIce::initialize(argc, argv);

	Ice::ObjectPrx tempmyMotors = ice->propertyToProxy("automata.myMotors.Proxy");
	if (tempmyMotors == 0) {
		throw "cannot create proxy from automata.myMotors.Proxy";
	}
	myMotors = jderobot::MotorsPrx::checkedCast(tempmyMotors);
	if (myMotors == 0) {
		throw "invalid proxy automata.myMotors.Proxy";
	}
	std::cout << "myMotors is connected" << std::endl;
}

void Interfaces::destroyProxies() {
	if (ice != 0) {
		ice->destroy();
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
	system("./trial10_runtime.py");
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
		pthread_create(&guiThread, NULL, &runGui, NULL);		runTimeGui = new RunTimeGui();

	}
	State* state0 = new State0(0, true, &interfaces, 100, NULL, runTimeGui);
	State* state1 = new State1(1, true, &interfaces, 100, state0, runTimeGui);
	State* state2 = new State2(2, false, &interfaces, 100, state0, runTimeGui);
	State* state3 = new State3(3, true, &interfaces, 100, state1, runTimeGui);
	State* state4 = new State4(4, false, &interfaces, 100, state1, runTimeGui);

	Transition* tran1 = new Tran1(1, 2, 10000);	state1->addTransition(tran1);
	Transition* tran5 = new Tran5(5, 1, 5000);	state2->addTransition(tran5);
	Transition* tran2 = new Tran2(2, 4, 1000);	state3->addTransition(tran2);
	Transition* tran4 = new Tran4(4, 3, 1000);	state4->addTransition(tran4);

	state0->startThread();
	state1->startThread();

	state0->join();
	state1->join();
}
