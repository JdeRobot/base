/*
 * gpio_reader.h
 *
 *  Created on: 6 de abr. de 2016
 *      Author: roberto
 */

#ifndef GPIO_READER_H_
#define GPIO_READER_H_

#include <boost/thread/thread.hpp>
#include "sharer.h"

namespace EMSensor {

class GPIO_reader{

public:

	/** Default constructor.
	 *
	 */
	GPIO_reader(Sharer* sharer);

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

}



#endif /* GPIO_READER_H_ */
