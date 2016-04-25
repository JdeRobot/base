/*
 * pwm_analyzer.h
 *
 *  Created on: 6 de abr. de 2016
 *      Author: roberto
 */

#ifndef PWM_ANALYZER_H_
#define PWM_ANALYZER_H_

#include <boost/thread/thread.hpp>
#include "sharer.h"

namespace EMSensor {

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


    void stop();

protected:
    /** Main analyzer loop.
     *
     * This method implements the actual analyzer loop, executing it while the
     * stop flag isn't active.
     */
    void run();

    boost::thread thread;   ///< Thread holder.
    Sharer* sharer;			///< Shared memory object.
    bool _stop;				///< Thread stop flag.
    
    unsigned int maxRestCount; ///< Maximum number of samples without variation.
};

}



#endif /* PWM_ANALYZER_H_ */
