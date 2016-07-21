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
 
#include <wiringPi.h>
#include "gpio_reader.h"

#define cycle 0.05

namespace EMSensor {

int GPIO_reader::GPIO_pin = -1;
void GPIO_reader::init_GPIO(int input_pin){
	if(GPIO_pin < 0){
		// Setup GPIO - needs root privileges or will fail
		wiringPiSetup();
		pinMode (input_pin, INPUT);
		pullUpDnControl (input_pin, PUD_DOWN);
		GPIO_pin = input_pin;
	}
}



GPIO_reader::GPIO_reader(Sharer* sharer){
	this->_stop = false;
	this->sharer = sharer;

}

void GPIO_reader::run(){
    struct timeval a, b;	// Time difference operand holders (time request)
    long totalb, totala;	// Time difference operand holders (actual operands)
    long diff;				// Time difference result

    struct timeval stamp;	// Time difference operand holders (time request)
    Sharer::DataValue dv;	// Data/time value to store
    Sharer::DataValue last_dv;	// Last DT

    int datum;      // Data recovered
    				
    last_dv.d = 0;
    last_dv.t = 0;    

	while(!this->_stop){
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        datum =  digitalRead(GPIO_pin);
        gettimeofday(&stamp, NULL);

        dv.d = datum;
        dv.t = stamp.tv_sec * 1000000 + stamp.tv_usec;
        
       
        sharer->writeBufferData(dv);



        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;
        diff = (diff < 0 || diff > cycle) ? cycle : cycle - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
//        if (diff < 33)
//            usleep(33 * 1000);
	}
}


void GPIO_reader::start(){
	thread = boost::thread(&GPIO_reader::run, this);
}

void GPIO_reader::join(){
	thread.join();
}

void GPIO_reader::stop(){
        this->_stop = true;
}

GPIO_reader::~GPIO_reader(){
	_stop = true;
	thread.join();

}

}


