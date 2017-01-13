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

#ifndef JDEROBOTCOMM_CAMERAICECLIENT_H_
#define JDEROBOTCOMM_CAMERAICECLIENT_H_

#include <IceUtil/IceUtil.h>
//#include <iostream>
#include <Ice/Ice.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <jderobot/camera.h>
#include <cv.h>
//#include <sstream>
//#include <fstream>
//#include <climits>
#include <log/Logger.h>
#include <jderobot/types/image.h>
#include <jderobot/comm/interfaces/cameraClient.hpp>
#include <zlib.h>

namespace JdeRobotComm {

class CameraIceClient: public IceUtil::Thread, public JdeRobotComm::CameraClient {
public:
	CameraIceClient(Ice::CommunicatorPtr ic, std::string prefix);
	virtual ~CameraIceClient();
	virtual void run();

	//callbacks
	virtual JdeRobotTypes::Image getImage();
	virtual int getRefreshRate();


	void pause();
	void resume();
	void reset();
	void stop_thread();
	bool getPause(){return pauseStatus;};

	jderobot::ImageFormat getImageFormat();
	void setImageFormat (std::string format);


private:
	jderobot::CameraPrx prx;
	long long int cycle;

	IceUtil::Mutex controlMutex;
	std::string prefix;

	int refreshRate;
	bool pauseStatus;

	IceUtil::Cond semWait;
	std::string mImageFormat;

};

} /* namespace JdeRobotComm */
#endif /* JDEROBOTCOMM_CAMERAICECLIENT_H_ */