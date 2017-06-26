/*
 *  Copyright (C) 2010 Eduardo Perdices García
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Authors : Eduardo Perdices García <eperdices@gsyc.es>,
 *             Jose María Cañas Plaza <jmplaza@gsyc.es>
 *
 */

#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/ptmotors.h>

#include "easyiceconfig/EasyIce.h" 

using namespace std;

int main(int argc, char** argv){

	Ice::CommunicatorPtr ic;
	int status = 0;


	std::cout << "init " << std::endl;

	try{
		ic = EasyIce::initialize(argc,argv);
		/*Get to giraffe server*/
		Ice::ObjectPrx base1 = ic->propertyToProxy("test.pantilt.Proxy");
		if (0==base1)
			throw "Could not create proxy to Pan Tilt";

		/*cast to JointMotorPrx*/
		jderobot::PTMotorsPrx ptprx = jderobot::PTMotorsPrx::checkedCast(base1);
		if (0==ptprx)
			throw "Invalid proxy";

		
		while(true){
			jderobot::PTMotorsDataPtr data=new jderobot::PTMotorsData();
			data->panSpeed = 10;
			data->pan = 100;
			data->tiltSpeed = 10;
			data->tilt= 30 ;

			ptprx->setPTMotorsData(data);
			
			usleep(1000000);

			data->panSpeed = 10;
			data->pan = -100;
			data->tiltSpeed = 10;
			data->tilt = -30 ;

			ptprx->setPTMotorsData(data);
			
			usleep(10000000);
		}
	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
		status = 1;
	} catch (const char* msg) {
		std::cerr << msg << std::endl;
		status = 1;
	}

	if (ic)
		ic->destroy();
	return status;
}