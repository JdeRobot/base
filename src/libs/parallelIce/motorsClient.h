/*
 *  Copyright (C) 1997-2013 JDE Developers TeamkinectViewer.camRGB
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
 *  Author : Jose María Cañas <jmplaza@gsyc.es>
			Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>

 */

#ifndef MOTORSCLIENT_H_
#define MOTORSCLIENT_H_

#include <IceUtil/IceUtil.h>
#include <iostream>
#include <Ice/Ice.h>
#include <jderobot/motors.h>
#include <cv.h>
#include <sstream>
#include <fstream>
#include <logger/Logger.h>
#include <boost/shared_ptr.hpp>

namespace jderobot {


class motorsClient {
public:
	motorsClient(Ice::CommunicatorPtr ic, std::string prefix, bool debug = false);
	virtual ~motorsClient();

	void setV(float v);
    void setW(float w);
    float getMaxW();
    float getMaxV();
    void sendVelocities();
    void sendV(float v);
    void sendW(float w);
   	bool hasProxy();


private:
	std::string prefix;
	float v;
	float w;
	float maxV;
	float maxW;


	jderobot::MotorsPrx prx;
	IceUtil::Mutex controlMutex;
	bool debug;
	bool _done;

};

	typedef boost::shared_ptr<motorsClient> MotorsClientPtr;


} /* namespace jderobot */
#endif /* LASERCLIENT_H_ */
