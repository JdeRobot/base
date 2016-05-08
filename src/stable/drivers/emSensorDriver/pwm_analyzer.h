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

#ifndef PWM_ANALYZER_H_
#define PWM_ANALYZER_H_

#include <boost/thread/thread.hpp>
#include "sharer.h"

namespace EMSensor {

/** Signal analysis thread.
 *
 * This class defines a thread dedicated to signal analysis and
 * sending data via ICE interface.
 */
class PWM_analyzer {

public:
    /** Default constructor.
     *
     *  @param sharer instance of Sharer object for data interchange.
     *  @param maxRestCount maximum number of samples without variation for the same wave.
     */
    PWM_analyzer(Sharer* sharer, unsigned int maxRestCount);

    /** Default destructor.
     *
     * @note this destructor waits for the related thread to finalize.
     */
    virtual ~PWM_analyzer();

    /** Starts the analyzer thread.
     *
     * This method ends the initialization of the analyzer thread and calls the
     * main loop for actual run.
     */
    void start();

    /** Waits until the analyzer thread finishes.
     *
     */
    void join();

    /** Signal a stop command to the associated thread, ending its work safely.
     *
     */
    void stop();

protected:
    /** Main analyzer loop.
     *
     * This method implements the actual analyzer loop, executing it while the
     * stop flag isn't active.
     */
    void run();

    boost::thread thread;       ///< Thread holder.
    Sharer* sharer;             ///< Shared memory object.
    bool _stop;                 ///< Thread stop flag.
    
    unsigned int maxRestCount; ///< Maximum number of samples without variation.
};

}



#endif /* PWM_ANALYZER_H_ */
