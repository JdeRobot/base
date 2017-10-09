#ifndef trial10_H
#define trial10_H

#include <state.h>
#include <temporaltransition.h>
#include <conditionaltransition.h>
#include <easyiceconfig/EasyIce.h>

#include <jderobot/motors.h>

class Interfaces {
public:
	Ice::CommunicatorPtr ice;
	jderobot::MotorsPrx myMotors;

	virtual void connectProxies(int argc, char* argv[]);
	virtual void destroyProxies();
};

class State0 : public State {
public:
	Interfaces* interfaces;
	State0(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):
		State(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}
	virtual void runCode();
};

class State1 : public State {
public:
	Interfaces* interfaces;
	State1(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):
		State(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}
	virtual void runCode();
};

class State2 : public State {
public:
	Interfaces* interfaces;
	State2(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):
		State(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}
	virtual void runCode();
};

class State3 : public State {
public:
	Interfaces* interfaces;
	State3(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):
		State(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}
	virtual void runCode();
};

class State4 : public State {
public:
	Interfaces* interfaces;
	State4(int id, bool initial, Interfaces* interfaces, int cycleDuration, State* parent, RunTimeGui* gui):
		State(id, initial, cycleDuration, parent, gui) {this->interfaces = interfaces;}
	virtual void runCode();
};

class Tran1 : public TemporalTransition {
	public:
	Tran1(int id, int destId, int elapsedTime):TemporalTransition(id, destId, elapsedTime) {}
	virtual void runCode();
};

class Tran5 : public TemporalTransition {
	public:
	Tran5(int id, int destId, int elapsedTime):TemporalTransition(id, destId, elapsedTime) {}
	virtual void runCode();
};

class Tran2 : public TemporalTransition {
	public:
	Tran2(int id, int destId, int elapsedTime):TemporalTransition(id, destId, elapsedTime) {}
	virtual void runCode();
};

class Tran4 : public TemporalTransition {
	public:
	Tran4(int id, int destId, int elapsedTime):TemporalTransition(id, destId, elapsedTime) {}
	virtual void runCode();
};

#endif