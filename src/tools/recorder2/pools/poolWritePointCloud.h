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

#ifndef POOLWRITEPOINTCLOUD_H_
#define POOLWRITEPOINTCLOUD_H_

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <stdio.h>
#include <time.h>
#include <jderobot/pointcloud.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <fstream>
#include "PoolPaths.h"

namespace recorder {

class poolWritePointCloud: public RecorderPool, public PoolPaths  {
public:
	poolWritePointCloud(Ice::ObjectPrx prx, int freq, size_t poolSize, int pclID,const std::string& baseLogPath);
	virtual ~poolWritePointCloud();
	void consumer_thread();
	void producer_thread();

	virtual void* consumer_thread_imp();
	virtual void* producer_thread_imp();


private:
	std::vector<jderobot::RGBPointsPCL > pointCloud;
	jderobot::pointCloudPrx pointCloudPrx;
};

	typedef boost::shared_ptr<poolWritePointCloud> poolWritePointCloudPtr;

} /* namespace recorder */
#endif /* POOLWRITEPOINTCLOUD_H_ */
