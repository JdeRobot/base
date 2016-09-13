#ifndef MOTORSI_H
#define MOTORSI_H

#include <jderobot/motors.h>
#include <turtlebot/turtlebotcontrol.hh>

namespace turtlebot{
namespace interfaces{

class MotorsI : virtual public jderobot::Motors {
public:

    MotorsI(turtlebot::TurtlebotControl* const control);
    virtual ~MotorsI();

    virtual float getV(const Ice::Current&);
    virtual float getW(const Ice::Current&);
    virtual float getL(const Ice::Current&);
    virtual Ice::Int setV(Ice::Float v, const Ice::Current&);
    virtual Ice::Int setW(Ice::Float _w, const Ice::Current&);
    virtual Ice::Int setL(Ice::Float l, const Ice::Current&);

private:
    turtlebot::TurtlebotControl* const control;
}; // end class MotorsI

}} //NS

#endif // MOTORSI_H
