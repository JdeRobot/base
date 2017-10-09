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

#ifndef JDEROBOTCOMM_RGBDICECLIENT_H_
#define JDEROBOTCOMM_RGBDICECLIENT_H_

#include <IceUtil/IceUtil.h>
#include <Ice/Ice.h>
#include <jderobot/rgbd.h>
#include <logger/Logger.h>
#include <jderobotutil/utils/CameraUtils.h>
#include <jderobot/types/rgbd.h>
#include <jderobot/comm/communicator.hpp>
#include <jderobot/comm/interfaces/rgbdClient.hpp>

namespace Comm {

class RgbdIceClient: public IceUtil::Thread, public Comm::RgbdClient {
public:
	RgbdIceClient(Comm::Communicator* jdrc, std::string prefix);
	virtual ~RgbdIceClient();
	virtual void run();

	//callbacks
	virtual JdeRobotTypes::Rgbd getRgbd();
	virtual int getRefreshRate();


	void pause();
	void resume();
	void stop_thread();
	bool getPause(){return pauseStatus;};

	jderobot::ImageFormat getImageFormat();
	void setImageFormat (std::string format);


private:
	jderobot::rgbdPrx prx;
	long long int cycle;

	IceUtil::Mutex controlMutex;
	std::string prefix;

	bool pauseStatus;

	IceUtil::Cond semWait;
	std::string mImageFormat;

};

} /* namespace Comm */
#endif /* JDEROBOTCOMM_RGBDICECLIENT_H_ */