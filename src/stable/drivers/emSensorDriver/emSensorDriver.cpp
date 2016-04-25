#include <iostream>
#include <signal.h>
#include <boost/lexical_cast.hpp>

// ICE utils includes
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <wiringPi.h>

#include "sharer.h"
#include "gpio_reader.h"
#include "pwm_analyzer.h"


Ice::CommunicatorPtr ic;    ///< ICE main communicator for this driver.
bool finish;                ///< Shutdown driver flag.

/** SIGINT signal handler for the driver.
 *
 *  This function handles the SIGINT signal to issue a shutdown order
 *  to the driver infrastructure.
 *  @param s signal code.
 */
void exitApplication(int s){

    std::cout << "Shutdown signal sent" << std::endl;
	finish=true;
	ic->shutdown();
    std::cout << "Shutdown signal confirmed" << std::endl;

}

/** Driver entry point.
 *
 * @param argc number of arguments passed to the driver.
 * @param argv pointer to the arguments passed to the driver.
 * @return exit code.
 */
int main(int argc, char** argv){
    using namespace EMSensor;
    Sharer* sharer;
    GPIO_reader* gpio_reader;
    PWM_analyzer* pwm_analyzer;

    int GPIO_port = 7;
    unsigned int maxRestCount = 100; 

    // Setup signal handler
    finish=false;
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exitApplication;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    std::string componentPrefix("emSensorDriver");

    try{
        //-----------------ICE-----------------//
        ic = Ice::initialize(argc, argv);

        Ice::PropertiesPtr prop = ic->getProperties();
        try {
            GPIO_port = boost::lexical_cast<int>(
                prop->getPropertyWithDefault(componentPrefix+".IO.port", "7"));
        
        } catch (boost::bad_lexical_cast&) {
            std::cerr << "Wrong value for "
                    << componentPrefix+".IO.port"
                    << "using default (" 
                    << GPIO_port << ")"
                    << std::endl;
        }
        
        try {
            maxRestCount = boost::lexical_cast<unsigned int>(
                prop->getPropertyWithDefault(componentPrefix+".Signal.maxRestCount", "100"));
        
        } catch (boost::bad_lexical_cast&) {
            std::cerr << "Wrong value for "
                    << componentPrefix+".Signal.maxRestCount"
                    << "using default (" 
                    << maxRestCount << ")"
                    << std::endl;
        }
        std::string Endpoints = prop->getPropertyWithDefault(componentPrefix+".emSensor.Endpoints",
            "default -h localhost -p 9090" );
        
        std::cout << "Config parsed" << std::endl;
        
        sharer = new Sharer();

        Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints("emSensor",
        		Endpoints);
        adapter->add(sharer->interface, ic->stringToIdentity(componentPrefix));
        
        adapter->activate();
        std::cout << "Adapter ready: " << Endpoints << std::endl;
        //--------------END ICE---------------//

        GPIO_reader::init_GPIO(GPIO_port);

        gpio_reader = new GPIO_reader(sharer);
        pwm_analyzer = new PWM_analyzer(sharer, maxRestCount);

        pwm_analyzer->start();
        gpio_reader->start();

        ic->waitForShutdown();
	}
	catch (const Ice::Exception& ex) {
			std::cerr << ex << std::endl;
			return 1;
	}
	catch (const char* msg) {
			std::cerr <<"Error :" << msg << std::endl;
			return 1;
	}
	
	if(pwm_analyzer)
    	pwm_analyzer->stop();
	if(gpio_reader)    	
    	gpio_reader->stop();
	if(pwm_analyzer)
    	pwm_analyzer->join();
	if(gpio_reader)
    	gpio_reader->join();



	if(ic){
		try{
			ic->destroy();
		}catch(const Ice::Exception& e){
			std::cerr << e << std::endl;
			return 1;
		}
	}
	


 
  return 0 ;
}

