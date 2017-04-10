/*
*  Copyright (C) 1997-2013 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Authors : Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>,
 *             Jose María Cañas Plaza <jmplaza@gsyc.es>
 *
 */

#ifndef POOLWRITELASERS_H_
#define POOLWRITELASERS_H_

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <stdio.h>
#include <time.h>
#include <jderobot/laser.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <fstream>
#include "RecorderPool.h"
#include "PoolPaths.h"


namespace recorder{

class LaserD
{
public:
    std::vector<int> values;
    float minAngle = 0;
    float maxAngle = 0;
    int minRange = 0;
    int maxRange = 0;
};



class poolWriteLasers: public RecorderPool, public PoolPaths  {
public:
	poolWriteLasers(Ice::ObjectPrx prx, int freq, size_t poolSize, int laserID, const std::string& baseLogPath);
	virtual ~poolWriteLasers();
	void consumer_thread();
	void producer_thread();

    virtual void* consumer_thread_imp();
    virtual void* producer_thread_imp();


private:
	std::vector<LaserD> lasers;
	jderobot::LaserPrx laserPrx;

	//threads

};

    typedef boost::shared_ptr<poolWriteLasers> poolWriteLasersPtr;
} //NAMESPACE

#endif /* POOLWRITELASERS_H_ */
