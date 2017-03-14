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

#ifndef SRC_DRIVERS_EMSENSORDRIVER_SHARER_H_
#define SRC_DRIVERS_EMSENSORDRIVER_SHARER_H_

#include <boost/circular_buffer.hpp>
#include <boost/container/list.hpp>
#include <boost/thread/mutex.hpp>
#include "emSensorI.h"

namespace EMSensor {

/** Class for controlled memory sharing between threads.
 *
 *  This class defines methods for safe memory sharing, as auxiliary
 *  type definitions for the rest of the classes.
 *  
 */
class Sharer{
 public:
    /** Timestamped adquired data values.
     */
    struct DataValue{
        int d;              ///< Datum value.
        unsigned long t;    ///< Timestamp value.
    };

    typedef jderobot::State StateI;     ///< EM sensor possible states.

    /// Default constructor.
    Sharer();

    /** Writes a new value into the adquisition buffer thread-safely.
     *
     * @param val Adquired value to be written into the adquisition buffer.
     */
    void writeBufferData(DataValue val);

	/** Reads all data stored into the adquisition buffer thread-safely.
	 *
	 *  This method extract current data stored into the buffer, leaving
	 *  it empty.
	 *
	 *  @return Pointer to a list of the recovered values, to be destroyed
	 *          by the caller.
	 */
    boost::container::list<DataValue>* readAllBufferData();


    /// Default destructor.
    virtual ~Sharer();

    /// ICE interface pointer.
    EMSensorI* interface;

 private:
    mutable boost::mutex synch;  ///< Mutex for thread-safe access to internal data.

    boost::circular_buffer<DataValue> data_buffer;  ///< Data adquisition buffer.
};

}  // namespace EMSensor

#endif  // SRC_DRIVERS_EMSENSORDRIVER_SHARER_H_
