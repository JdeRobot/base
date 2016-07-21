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

#ifndef SRC_DRIVERS_EMSENSORDRIVER_GPIO_READER_H_
#define SRC_DRIVERS_EMSENSORDRIVER_GPIO_READER_H_

#include <boost/thread/thread.hpp>
#include "sharer.h"

namespace EMSensor {

/** Data adquisition thread.
 *
 * This class defines a thread dedicated to adquire data from
 * a General Purpose Input/Output port. 
 */
class GPIO_reader{
 public:
	/** Default constructor.
	 *
	 */
    explicit GPIO_reader(Sharer* sharer);

        /** Default destructor.
	 *
	 * @note this destructor waits for the related thread to finalize.
	 */
    virtual ~GPIO_reader();

	/** Starts the reader thread.
	 *
	 * This method ends the initialization of the analyzer thread and calls the
	 * main loop for actual run.
	 */
    void start();

	/** Waits until the reader thread finishes.
	 *
	 */
    void join();

    /** Signal a stop command to the associated thread, ending its work safely.
     *
     */
    void stop();

	/** Initializes GPIO for reading at required pin.
	 *
	 * This static method initializes the WiringPi library in order to set
	 * the GPIO desired port for reading.
	 *
	 * @note This method requires privileges for low-level GPIO access.
	 *
	 * @param input_pin Desired pin for reading according to WiringPi.
	 *
	 */
    static void init_GPIO(int input_pin);


 protected:
    /** Main acquisition loop.
     *
     * This method implements the actual acquisition loop, executing it while the
     * stop flag isn't active.
     */
    void run();

    static int GPIO_pin;    ///< GPIO input pin.

    boost::thread thread;   ///< Thread holder.
    Sharer* sharer;         ///< Shared memory object.
    bool _stop;              ///< Thread stop flag.
};

}  // namespace EMSensor

#endif  // SRC_DRIVERS_EMSENSORDRIVER_GPIO_READER_H_
