#ifndef LASERMANAGER_H
#define LASERMANAGER_H

#include<jderobot/laser.h>

class LaserManager
{
public:
    LaserManager(){}
    virtual ~LaserManager(){}
    virtual jderobot::LaserData* getLaserData()=0;    // "=0" part makes this method pure virtual, and
                                 // also makes this class abstract.
    //virtual void update()=0;
};


#endif // LASERMANAGER_H
