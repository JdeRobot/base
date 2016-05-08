/*
 *  Copyright (C) 1997-2016 JDE Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *  Authors : 
 *       Luis Roberto Morales Iglesias <lr.morales.iglesias@gmail.com>	
 */

#ifndef EM_SENSOR_H_
#define EM_SENSOR_H_
 
#include <boost/thread/mutex.hpp>
#include <jderobot/emSensor.h>

namespace EMSensor {

class EMSensorI :  virtual public jderobot::EMSensor {
public:
	EMSensorI();

	void setData(float t, float d, jderobot::State status);
	virtual jderobot::EMSensorDataPtr getEMSensorData(const ::Ice::Current&);

protected:
	mutable boost::mutex synch; ///< Mutex for thread-safe access to internal data.

	float time_stamp;
	float distance;
	jderobot::State status;
};

} /* namespace EMSensor */

#endif /* EM_SENSOR_H_ */
