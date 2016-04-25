/*
 * ProximitySensor.h
 *
 *  Created on: 7 de abr. de 2016
 *      Author: roberto
 */

#ifndef PROXIMITY_SENSOR_H_
#define PROXIMITY_SENSOR_H_

#include <boost/thread/mutex.hpp>
#include <jderobot/proximitySensor.h>

namespace EMSensor {

class ProximitySensorI :  virtual public jderobot::ProximitySensor {
public:
	ProximitySensorI();

	void setData(float t, float d, jderobot::State status);
	virtual jderobot::ProximitySensorDataPtr getProximitySensorData(const ::Ice::Current&);
	//virtual ~ProximitySensorI();

protected:
	mutable boost::mutex synch; ///< Mutex for thread-safe access to internal data.

	float time_stamp;
	float distance;
	jderobot::State status;
};

} /* namespace EMSensor */

#endif /* PROXIMITY_SENSOR_H_ */
