/*
 * pwm_analyzer.cpp
 *
 *  Created on: 6 de abr. de 2016
 *      Author: Luis Roberto Morales Iglesias
 */
#include "pwm_analyzer.h"

#define cycle 50

namespace EMSensor {

PWM_analyzer::PWM_analyzer(Sharer* sharer, unsigned int maxRestCount=20){
	this->_stop = false;
	this->sharer = sharer;
    this->maxRestCount = maxRestCount;
}

void PWM_analyzer::run(){
    struct timeval a, b;	// Time difference operand holders (time request)
    long totalb, totala;	// Time difference operand holders (actual operands)
    long diff;			// Time difference result
    
    unsigned long changeCount = 0;
    unsigned long sampleCount = 0;
    unsigned int restCount = 0;
    float avgTsample = 0;       // Average sample period
    float tDistance = 0;        // Current time diff. between signal variations
    float lastTdistance = 0;    // Last time diff.  between signal variations
    float freq = 0;             // Current frequency of signal.
    float lastFreq = 0;         // Last frequency of signal.
    
    Sharer::DataValue lastDatum;
    lastDatum.d = -1;
    lastDatum.t = 0;

	while(!this->_stop){
        gettimeofday(&a, NULL);
        totala = a.tv_sec * 1000000 + a.tv_usec;

        boost::container::list<Sharer::DataValue>* data = sharer->readAllBufferData();
        
        //TODO: Signal treatment here:
        for (boost::container::list<Sharer::DataValue>::iterator it = data->begin() ; it != data->end(); ++it){
            Sharer::DataValue datum = (*it);
            
            float Tsample = datum.t -lastDatum.t;
            avgTsample = avgTsample*0.5+Tsample*0.5;
            
            sampleCount++;
            
            // Signal variation
            if(datum.d != lastDatum.d){
                // After quiet zone
                if(restCount > 0){
                     lastTdistance = tDistance;
                     tDistance = restCount;
                     restCount = 0;
                }
                changeCount++;
            }else{
                restCount++;
            }
            
            if(restCount > this->maxRestCount){
                if(changeCount > 0){
                    float t    = sampleCount*Tsample;
                    lastFreq = freq;
                    freq = changeCount/t;
                    changeCount = 0;
                    sampleCount = 0;
                }
            }
                
            lastDatum = datum;
        }
        

        sharer->interface->setData(totala, 0, Sharer::StateI::Error);

        delete data;

        gettimeofday(&b, NULL);
        totalb = b.tv_sec * 1000000 + b.tv_usec;
        diff = (totalb - totala) / 1000;
        diff = (diff < 0 || diff > cycle) ? cycle : cycle - diff;

        /*Sleep Algorithm*/
        usleep(diff * 1000);
        if (diff < 33)
            usleep(33 * 1000);
	}
}


void PWM_analyzer::start(){
	thread = boost::thread(&PWM_analyzer::run, this);
}

void PWM_analyzer::stop(){
        this->_stop = true;
}

void PWM_analyzer::join(){
	thread.join();
}

PWM_analyzer::~PWM_analyzer(){
	_stop = true;
	thread.join();

}

}



