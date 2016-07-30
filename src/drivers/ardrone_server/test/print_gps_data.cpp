/*
 *  Copyright (C) 1997-2015 JDE Developers Team
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
 *       Victor Arribas Raigadas <.varribas.urjc@gmail.com>
 */

#include <stdlib.h>
#include <iostream>
#include <boost/format.hpp>
#include <Ice/Ice.h>
#include "easyiceconfig/EasyIce.h"

#include <navdatagps.h>

void print_data(ardrone::NavdataGPSData data);

int main(int argc, char* argv[]) {
	Ice::CommunicatorPtr ic = EasyIce::initialize(argc, argv);

	//Ice::ObjectPrx prx = ic->propertyToProxy("ArDrone.NavdataGPS.Proxy");
	Ice::ObjectPrx prx = ic->stringToProxy("ardrone_navdatagps:tcp -h 0.0.0.0 -p 9993");
	if (0 == prx){
		std::cerr << "NavdataGPS configuration not specified" << std::endl;
		return 1;
	}

	ardrone::NavdataGPSPrx gpsprx;
	try{
		gpsprx = ardrone::NavdataGPSPrx::checkedCast(prx);
	}catch(Ice::ConnectFailedException e){
		std::cerr << e << std::endl;
		return 2;
	}

	//for(char c; std::cin >> c; c <= 0){
	while (!ic->isShutdown()){
		ardrone::NavdataGPSData data = gpsprx->getNavdataGPS();
		print_data(data);
		sleep(1);
	}

	ic->waitForShutdown();
	std::cout << "Ice clean exit" << std::endl;

	return 0;
}


void print_data(ardrone::NavdataGPSData data){
	static boost::format data_fmt(
"GPS Data\n\
├── status:\n\
│   ├── connected: %d\n\
│   ├── firmware status: %d\n\
│   ├── gps state: %d\n\
│   └── nbsat: %d\n\
├── COORDINATES:     %.4f %.4f %.4f\n\
├── Go Home coords:  %.4f %.4f\n\
└── timestamp: %d\n"
	);

	std::cout << data_fmt
		%data.isGpsPlugged % data.firmwareStatus
		% data.gpsState % data.nbsat
		% data.latitude % data.longitude % data.elevation
		% data.lat0 %data.long0
		% data.lastFrameTimestamp
	;
}
