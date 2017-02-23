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

#include "jderobot/comm/ice/cameraIceClient.hpp"


namespace JdeRobotComm {


CameraIceClient::CameraIceClient(Ice::CommunicatorPtr ic, std::string prefix) {

	this->prefix=prefix;
	Ice::PropertiesPtr prop;
	prop = ic->getProperties();
	Ice::ObjectPrx baseCamera;
	this->refreshRate=0;
	this->mImageFormat.empty();

	

	int fps=prop->getPropertyAsIntWithDefault(prefix+".Fps",30);
	this->cycle=(float)(1/(float)fps)*1000000;
	try{
		baseCamera = ic->propertyToProxy(prefix+".Proxy");
		if (0==baseCamera){
			this->on = false;
			throw prefix + "Could not create proxy with Camera";
		}
		else {
			this->prx= jderobot::CameraPrx::checkedCast(baseCamera);
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
		jderobot::Logger::getInstance()->error(prefix + " Not camera provided");
	}

	//check if default format is defined
	std::string definedFormat=prop->getPropertyWithDefault(prefix+".Format", "RGB8");

	// Discover what format are supported.
	jderobot::ImageFormat formats = this->prx->getImageFormat();

	std::vector<std::string>::iterator it;
  it = std::find(formats.begin(), formats.end(), definedFormat);
  if (it==formats.end()){
    it = std::find(formats.begin(), formats.end(), colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);

    if (it != formats.end())
    {
      this->mImageFormat = colorspaces::ImageRGB8::FORMAT_RGB8.get()->name;
      it = std::find(formats.begin(), formats.end(), colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name);
      if (it != formats.end())
        this->mImageFormat = colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name;
    }
    else
    {
      it = std::find(formats.begin(), formats.end(), colorspaces::ImageRGB8::FORMAT_DEPTH8_16.get()->name);
      if (it != formats.end())
      {
        this->mImageFormat = colorspaces::ImageRGB8::FORMAT_DEPTH8_16.get()->name;
        it = std::find(formats.begin(), formats.end(), colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name);
        if (it != formats.end())
          this->mImageFormat = colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name;
      }
      else{
        this->mImageFormat = colorspaces::ImageGRAY8::FORMAT_GRAY8.get()->name;
        it = std::find(formats.begin(), formats.end(), colorspaces::ImageGRAY8::FORMAT_GRAY8_Z.get()->name);
        if (it != formats.end())
          this->mImageFormat = colorspaces::ImageGRAY8::FORMAT_GRAY8_Z.get()->name;
      }
    }
  }
  else{
    this->mImageFormat = definedFormat;
  }

	jderobot::Logger::getInstance()->info("Negotiated format " + this->mImageFormat + " for camera " + this->prx->getCameraDescription()->name);

	jderobot::ImageDataPtr data = this->prx->getImageData(this->mImageFormat);

	this->pauseStatus=false;
}


CameraIceClient::~CameraIceClient() {
	this->on=false;
}


jderobot::ImageFormat CameraIceClient::getImageFormat()
{
	return (this->prx->getImageFormat());
}

void CameraIceClient::setImageFormat (std::string format)
{
	mImageFormat = format;

	jderobot::Logger::getInstance()->info("Changed format " + this->mImageFormat + " for camera " + this->prx->getCameraDescription()->name);
};


void CameraIceClient::reset(){
	this->prx->reset();
}

void CameraIceClient::pause(){
	this->pauseStatus=true;
}

void CameraIceClient::resume(){
	this->controlMutex.lock();
	this->pauseStatus=false;
	this->semWait.broadcast();
	this->controlMutex.unlock();
}


void
CameraIceClient::run(){
	jderobot::ImageDataPtr dataPtr;
	colorspaces::Image::FormatPtr fmt;
	IceUtil::Time last;

	int iterIndex = 0;
	int totalRefreshRate = 0;
	int refrRate = 0;

	JdeRobotTypes::Image img;

	last=IceUtil::Time::now();
	while (this->on){

		iterIndex ++;
		if (pauseStatus){
			IceUtil::Mutex::Lock sync(this->controlMutex);
			this->semWait.wait(sync);
		}

		try{

			dataPtr = this->prx->getImageData(this->mImageFormat);

			fmt = colorspaces::Image::Format::searchFormat(dataPtr->description->format);
			if (!fmt)
				throw "Format not supported";

			// Putting image data
			img.format = dataPtr->description->format;
			img.width = dataPtr->description->width;
			img.height = dataPtr->description->height;
			img.timeStamp = dataPtr->timeStamp.seconds + dataPtr->timeStamp.useconds * 1e-6;

			//creating image cv::mat
			if (dataPtr->description->format == colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name ||
					dataPtr->description->format == colorspaces::ImageRGB8::FORMAT_DEPTH8_16_Z.get()->name	)
			{

				size_t dest_len = dataPtr->description->width*dataPtr->description->height*3;
				size_t source_len = dataPtr->pixelData.size();

				unsigned char* origin_buf = (uchar*) malloc(dest_len);

				int r = uncompress((Bytef *) origin_buf, (uLongf *) &dest_len, (const Bytef *) &(dataPtr->pixelData[0]), (uLong)source_len);

				if(r != Z_OK) {
					fprintf(stderr, "[CMPR] Error:\n");
					switch(r) {
					case Z_MEM_ERROR:
						fprintf(stderr, "[CMPR] Error: Not enough memory to compress.\n");
						break;
					case Z_BUF_ERROR:
						fprintf(stderr, "[CMPR] Error: Target buffer too small.\n");
						break;
					case Z_STREAM_ERROR:    // Invalid compression level
						fprintf(stderr, "[CMPR] Error: Invalid compression level.\n");
						break;
					}
				}
				else
				{
					colorspaces::Image imageRGB(dataPtr->description->width,dataPtr->description->height,colorspaces::ImageRGB8::FORMAT_RGB8,&(origin_buf[0]));
					colorspaces::ImageRGB8 img_rgb888(imageRGB);//conversion will happen if needed
					cv::Mat(cvSize(img_rgb888.width,img_rgb888.height), CV_8UC3, img_rgb888.data).copyTo(img.data);
					
					img_rgb888.release();
				}


				if (origin_buf)
					free(origin_buf);

			}
			else if (dataPtr->description->format == colorspaces::ImageRGB8::FORMAT_RGB8.get()->name ||
					dataPtr->description->format == colorspaces::ImageRGB8::FORMAT_DEPTH8_16.get()->name  )
			{
				colorspaces::Image imageRGB(dataPtr->description->width,dataPtr->description->height,colorspaces::ImageRGB8::FORMAT_RGB8,&(dataPtr->pixelData[0]));
				colorspaces::ImageRGB8 img_rgb888(imageRGB);//conversion will happen if needed
				cv::Mat(cvSize(img_rgb888.width,img_rgb888.height), CV_8UC3, img_rgb888.data).copyTo(img.data);
				
				img_rgb888.release();
			}
			else if (dataPtr->description->format == colorspaces::ImageGRAY8::FORMAT_GRAY8_Z.get()->name) {
				//gay compressed
				size_t dest_len = dataPtr->description->width*dataPtr->description->height;
				size_t source_len = dataPtr->pixelData.size();

				unsigned char* origin_buf = (uchar*) malloc(dest_len);

				int r = uncompress((Bytef *) origin_buf, (uLongf *) &dest_len, (const Bytef *) &(dataPtr->pixelData[0]), (uLong)source_len);

				if(r != Z_OK) {
					fprintf(stderr, "[CMPR] Error:\n");
					switch(r) {
					case Z_MEM_ERROR:
						fprintf(stderr, "[CMPR] Error: Not enough memory to compress.\n");
						break;
					case Z_BUF_ERROR:
						fprintf(stderr, "[CMPR] Error: Target buffer too small.\n");
						break;
					case Z_STREAM_ERROR:    // Invalid compression level
						fprintf(stderr, "[CMPR] Error: Invalid compression level.\n");
						break;
					}
				}
				else
				{
					colorspaces::Image imageGray(dataPtr->description->width,dataPtr->description->height,colorspaces::ImageGRAY8::FORMAT_GRAY8,&(origin_buf[0]));
					colorspaces::ImageGRAY8 img_gray8(imageGray);//conversion will happen if needed

					cv::Mat(cvSize(img_gray8.width,img_gray8.height), CV_8UC1, img_gray8.data).copyTo(img.data);

					img_gray8.release();
				}


				if (origin_buf)
					free(origin_buf);
			}
			else if (dataPtr->description->format == colorspaces::ImageGRAY8::FORMAT_GRAY8.get()->name){
				colorspaces::Image imageGray(dataPtr->description->width,dataPtr->description->height,colorspaces::ImageGRAY8::FORMAT_GRAY8,&(dataPtr->pixelData[0]));
				colorspaces::ImageGRAY8 img_gray8(imageGray);//conversion will happen if needed
				
				cv::Mat(cvSize(img_gray8.width,img_gray8.height), CV_8UC1, img_gray8.data).copyTo(img.data);
				
				img_gray8.release();
			}
			else{
				//TODO raise exception
			}

			//end image cv::mat



		}
		catch(...){
			jderobot::Logger::getInstance()->warning(prefix +"error during request (connection error)");
			usleep(50000);

		}

		int process = (IceUtil::Time::now().toMicroSeconds() - last.toMicroSeconds());



		if (process > (int)cycle ){
			jderobot::Logger::getInstance()->warning("--------" + prefix + " adquisition timeout-");
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
			jderobot::Logger::getInstance()->info( "*** Counter reset");
		}

		this->controlMutex.lock();
		this->image = img;
		this->refreshRate = refrRate;
		this->controlMutex.unlock();

	}

	this->image.data.release();
}

void CameraIceClient::stop_thread()
{
	this->on=false;
}

JdeRobotTypes::Image CameraIceClient::getImage(){
	JdeRobotTypes::Image img;

	this->controlMutex.lock();
	img = this->image;
	this->controlMutex.unlock();

	return img;
}

int CameraIceClient::getRefreshRate(){
	int rr;
	this->controlMutex.lock();
	rr = this->refreshRate;
	this->controlMutex.unlock();

	return rr;
};

} /* namespace jderobot */
