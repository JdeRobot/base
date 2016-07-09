#ifndef LASERI_H
#define LASERI_H
#include <iostream>
#include <math.h>
#include <Ice/Ice.h>
#include <jderobot/laser.h>
#include <lasermanager.h>
#include <hokuyo/hokuyomanager.h>

namespace laser
{
	class LaserI: virtual public jderobot::Laser
	{
		public:
			LaserI(Ice::PropertiesPtr prop);
			virtual ~LaserI();
			virtual jderobot::LaserDataPtr getLaserData(const Ice::Current&);
		private:
			LaserManager *manager;

	};
}

#endif // LASERI_H


