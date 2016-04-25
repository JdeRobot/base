/*
 * sharer.cpp
 *
 *  Created on: 7 de abr. de 2016
 *      Author: roberto
 */

#include <boost/thread/lock_guard.hpp>
#include "sharer.h"

namespace EMSensor {

Sharer::Sharer(){
	this->data_buffer = boost::circular_buffer<DataValue>(1000);
	this->interface = new ProximitySensorI();
}


void Sharer::writeBufferData(DataValue val){
	boost::lock_guard<boost::mutex> lock(this->synch);
	this->data_buffer.push_back(val);

}

boost::container::list<Sharer::DataValue>* Sharer::readAllBufferData(){
	boost::lock_guard<boost::mutex> lock(this->synch);
	boost::container::list<DataValue>* list = new boost::container::list<DataValue>();
	while(!this->data_buffer.empty()){
                list->push_back(this->data_buffer.front());
                this->data_buffer.pop_front();

	}
	return list;

}

Sharer::~Sharer(){
	delete interface;
}

}
