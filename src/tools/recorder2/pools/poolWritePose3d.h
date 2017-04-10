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

#ifndef poolWritePose3d_H_
#define poolWritePose3d_H_

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <stdio.h>
#include <time.h>
#include <jderobot/pose3d.h>
#include <fstream>
#include "PoolPaths.h"


namespace recorder{

struct pose3d{
	float x;
	float y;
	float z;
	float h;
	float q0;
	float q1;
	float q2;
	float q3;
};


class poolWritePose3d: public RecorderPool, public PoolPaths  {
public:
	poolWritePose3d(Ice::ObjectPrx prx, int freq, size_t poolSize, int pose3dID, const std::string& baseLogPath);
	virtual ~poolWritePose3d();
	void consumer_thread();
	void producer_thread();

	virtual void* consumer_thread_imp();
	virtual void* producer_thread_imp();

private:
	std::vector<pose3d> pose3dVec;
	jderobot::Pose3DPrx pose3DPrx;


	//threads

};
	typedef boost::shared_ptr<poolWritePose3d> poolWritePose3dPtr;
} //NAMESPACE
#endif /* poolWritePose3d_H_ */
