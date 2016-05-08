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

#include <boost/thread/lock_guard.hpp>
#include "sharer.h"

namespace EMSensor {

Sharer::Sharer(){
	this->data_buffer = boost::circular_buffer<DataValue>(1000);
	this->interface = new EMSensorI();
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
