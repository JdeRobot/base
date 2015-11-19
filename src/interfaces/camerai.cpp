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
 *       Victor Arribas Raigadas <v.arribas.urjc@gmai.com>
 */

#include <quadrotor/interfaces/camerai.h>

using namespace quadrotor::interfaces;
using namespace jderobot;


CameraI::CameraI():
	imageDescription(new ImageDescription()),
	cameraDescription(new CameraDescription())
{
	std::cout << "Constructor CameraI" << std::endl;
}

CameraI::~CameraI(){
}


ImageDescriptionPtr
CameraI::getImageDescription(const Ice::Current& c){
    std::cout<<"CameraI::getImageDescription()"<<std::endl;
    if (!imageDescription)
        throw JderobotException("not initialized");

	return imageDescription;
}


ImageFormats
CameraI::getImageFormat(const Ice::Current& c) {
    std::cout<<"CameraI::getImageFormat()"<<std::endl;
    if (imageFormats.empty())
        throw JderobotException("not initialized");

	return imageFormats;
}


void
CameraI::getImageData_async (const jderobot::AMD_ImageProvider_getImageDataPtr& cb,const std::string& format, const Ice::Current& c){
    std::cout<<"CameraI::getImageData_async()"<<std::endl;
    if (std::find(imageFormats.begin(), imageFormats.end(), format) == imageFormats.end())
        throw JderobotException("format not supported"); // UnknownUserException: unknown = jderobot::JderobotException because it is not declared into .ice

    _getImageData_async(cb, format, c);
}

void
CameraI::_getImageData_async (const jderobot::AMD_ImageProvider_getImageDataPtr& cb,const std::string& format, const Ice::Current& c){
    //Non async response
    if (!imageData)
        throw DataNotExistException();

    cb->ice_response(imageData);
}


CameraDescriptionPtr
CameraI::getCameraDescription(const Ice::Current& c){
    std::cout<<"CameraI::getCameraDescription()"<<std::endl;
    if (!cameraDescription)
        throw new JderobotException("not initialized");

    return cameraDescription;
}
