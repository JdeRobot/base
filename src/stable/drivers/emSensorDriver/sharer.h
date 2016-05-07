/*
 * sharer.h
 *
 *  Created on: 6 de abr. de 2016
 *      Author: roberto
 */

#ifndef SHARER_H_
#define SHARER_H_

#include <boost/circular_buffer.hpp>
#include <boost/container/list.hpp>
#include <boost/thread/mutex.hpp>
#include "EMSensorI.h"

namespace EMSensor {

class Sharer{

public:
	struct DataValue{
		int d;	///< Datum value.
		unsigned long t;	///< Timestamp value.
	};

	typedef jderobot::State StateI;

	/// Default constructor.
	Sharer();

	void writeBufferData(DataValue val);
	boost::container::list<DataValue>* readAllBufferData();


	/// Default destructor.
	virtual ~Sharer();

	EMSensorI* interface;

private:

    mutable boost::mutex synch; ///< Mutex for thread-safe access to internal data.

    boost::circular_buffer<DataValue> data_buffer;
};
}



#endif /* SHARER_H_ */
