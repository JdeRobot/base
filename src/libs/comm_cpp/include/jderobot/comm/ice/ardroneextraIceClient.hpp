/*
 *  Copyright (C) 1997-2017 JDE Developers
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
 *
 *  Author : Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#ifndef JDEROBOTCOMM_ARDRONEEXTRAICECLIENT_H_
#define JDEROBOTCOMM_ARDRONEEXTRAICECLIENT_H_

#include <IceUtil/IceUtil.h>
#include <iostream>
#include <Ice/Ice.h>
#include <jderobot/ardroneextra.h>
#include <jderobot/comm/communicator.hpp>
#include <jderobot/comm/interfaces/ardroneextraClient.hpp>

namespace Comm {


class ArDroneExtraIceClient: public Comm::ArDroneExtraClient {
public:
	ArDroneExtraIceClient(Comm::Communicator* jdrc, std::string prefix);
	virtual ~ArDroneExtraIceClient();

	virtual void toggleCam();
	virtual void land();
	virtual void takeoff();
	virtual void reset();
	virtual void recordOnUsb(bool record);
	virtual void ledAnimation(int type,float duration, float req);
	virtual void flightAnimation(int type, float duration);
	virtual void flatTrim();



private:
	std::string prefix;
	jderobot::ArDroneExtraPrx prx;
	IceUtil::Mutex controlMutex;
	
	
};


} /* namespace jderobotcomm */
#endif /* JDEROBOTCOMM_ARDRONEEXTRAICECLIENT_H_ */