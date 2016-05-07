/*
 * ProximitySensor.h
 *
 *  Created on: 7 de abr. de 2016
 *      Author: roberto
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
