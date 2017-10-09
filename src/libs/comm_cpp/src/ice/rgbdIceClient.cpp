/*
 *  Copyright (C) 1997-2016 JDE Developers TeamkinectViewer.camRGB
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

#include "jderobot/comm/ice/rgbdIceClient.hpp"

#include <exception>


namespace Comm {


RgbdIceClient::RgbdIceClient(Comm::Communicator* jdrc, std::string prefix) {

	this->prefix=prefix;
	Ice::ObjectPrx baseRgbd;
	this->refreshRate=0;
	this->mImageFormat.empty();

	

	float fps=jdrc->getConfig().asFloatWithDefault(prefix+".Fps", 30);
	this->cycle=(1/fps)*1000000;
	try{
		std::string proxy = jdrc->getConfig().asString(prefix+".Proxy");
		baseRgbd = jdrc->getIceComm()->stringToProxy(proxy);
		if (0==baseRgbd){
			this->on = false;
			throw prefix + "Could not create proxy with Rgbd";
		}
		else {
			this->prx= jderobot::rgbdPrx::checkedCast(baseRgbd);
			this->on = true;
			if (0==this->prx){
				this->on = false;
				throw "Invalid " + prefix + ".Proxy";
			}
		}
	}catch (const Ice::Exception& ex) {
		std::cerr << ex << std::endl;
	}
	catch (const char* msg) {
		std::cerr << msg << std::endl;
		LOG(ERROR) <<prefix + " Not rgbd provided";
	}

	//check if default format is defined
	//std::string definedFormat=jdrc->getConfig().asStringWithDefault(prefix+".Format", "RGB8");

	//this->mImageFormat = RgbdUtils::negotiateDefaultFormat(this->prx,definedFormat);

	jderobot::rgbData data = this->prx->getData();

	this->pauseStatus=false;
}


RgbdIceClient::~RgbdIceClient() {
	this->on=false;
}


/*jderobot::ImageFormat RgbdIceClient::getImageFormat()
{
	return (this->prx->getImageFormat());
}

void RgbdIceClient::setImageFormat (std::string format)
{
	mImageFormat = format;

	LOG(INFO) <<"Changed format " + this->mImageFormat + " for rgbd " + this->prx->getRgbdDescription()->name;
};
*/

void RgbdIceClient::pause(){
	this->pauseStatus=true;
}

void RgbdIceClient::resume(){
	this->controlMutex.lock();
	this->pauseStatus=false;
	this->semWait.broadcast();
	this->controlMutex.unlock();
}


void
RgbdIceClient::run(){
	jderobot::rgbData data;
	IceUtil::Time last;

	int iterIndex = 0;
	int totalRefreshRate = 0;
	int refrRate = 0;

	JdeRobotTypes::Rgbd img;

	last=IceUtil::Time::now();
	while (this->on){

		iterIndex ++;
		if (pauseStatus){
			IceUtil::Mutex::Lock sync(this->controlMutex);
			this->semWait.wait(sync);
		}

		try{

			JdeRobotTypes::Image color;
			JdeRobotTypes::Image depth;
			
			data = this->prx->getData();

			
			color.data = CameraUtils::getImageFromCameraProxy(data.color);
			color.format = data.color->description->format;
			color.width = data.color->description->width;
			color.height = data.color->description->height;
			color.timeStamp = data.color->timeStamp.seconds + data.color->timeStamp.useconds * 1e-6;

			depth.data = CameraUtils::getImageFromCameraProxy(data.depth);
			depth.format = data.depth->description->format;
			depth.width = data.depth->description->width;
			depth.height = data.depth->description->height;
			depth.timeStamp = data.depth->timeStamp.seconds + data.depth->timeStamp.useconds * 1e-6;
		
			img.timeStamp = data.timeStamp.seconds + data.timeStamp.useconds * 1e-6;
			img.color = color;
			img.depth = depth;




		}
		catch(std::exception& e){
			LOG(WARNING) << prefix +"error during request (connection error): " << e.what() << std::endl;
			usleep(50000);

		}

		int process = (IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds());



		if (process > (int)cycle ){
			DLOG(WARNING) << "--------" + prefix + " adquisition timeout-";
		}
		else{
			int delay = (int)cycle - process;
			if (delay <1 || delay > (int)cycle)
				delay = 1;

			usleep(delay);
		}


		int rate =(int)(1000000/(IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds()));
		totalRefreshRate =  totalRefreshRate + rate;
		refrRate = totalRefreshRate / iterIndex;
		last=IceUtil::Time::now();

		if (iterIndex == INT_MAX) 
		{
			iterIndex = 0;
			DLOG(INFO) << "*** Counter reset";
		}

		this->controlMutex.lock();
		this->rgbd = img;
		this->refreshRate = refrRate;
		this->controlMutex.unlock();

	}

	this->rgbd.color.data.release();
}

void RgbdIceClient::stop_thread()
{
	this->on=false;
}

JdeRobotTypes::Rgbd RgbdIceClient::getRgbd(){
	JdeRobotTypes::Rgbd img;

	this->controlMutex.lock();
	img = this->rgbd;
	this->controlMutex.unlock();

	return img;
}

int RgbdIceClient::getRefreshRate(){
	int rr;
	this->controlMutex.lock();
	rr = this->refreshRate;
	this->controlMutex.unlock();

	return rr;
};

} /* namespace jderobot */
