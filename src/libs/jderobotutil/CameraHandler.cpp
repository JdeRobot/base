/*
 * CameraHandler.cpp
 *
 *  Created on: 29/6/2015
 *      Author: frivas
 */

#include <jderobotutil/CameraHandler.h>

namespace jderobot {

CameraHandler::CameraHandler(std::string propertyPrefix, Ice::CommunicatorPtr ic): prefix(propertyPrefix), imageConsumer() {
  imageDescription = (new jderobot::ImageDescription());
      cameraDescription = (new jderobot::CameraDescription());

      prop = Ice::PropertiesPtr(ic->getProperties());

      //fill cameraDescription
      cameraDescription->name = prop->getProperty(prefix+"Name");
      if (cameraDescription->name.size() == 0)
        throw "Camera name not configured";

      cameraDescription->shortDescription = prop->getProperty(prefix+"ShortDescription");
      cameraDescription->streamingUri = prop->getProperty(prefix+"StreamingUri");

      //fill imageDescription
      imageDescription->width = prop->getPropertyAsIntWithDefault(prefix+"ImageWidth",340);
      imageDescription->height = prop->getPropertyAsIntWithDefault(prefix+"ImageHeight",280);


}

CameraHandler::~CameraHandler() {
  // TODO Auto-generated destructor stub
}

std::string CameraHandler::getName () {
  return (cameraDescription->name);
}

jderobot::ImageDescriptionPtr CameraHandler::getImageDescription(const Ice::Current& c){
  return (imageDescription);
}

jderobot::CameraDescriptionPtr CameraHandler::getCameraDescription(const Ice::Current& c){
  return (cameraDescription);
}

Ice::Int CameraHandler::setCameraDescription(const jderobot::CameraDescriptionPtr &description, const Ice::Current& c) {
  return (0);
}

jderobot::ImageFormat CameraHandler::getImageFormat(const Ice::Current& c)
{
  jderobot::ImageFormat formats;

  formats.push_back(colorspaces::ImageRGB8::FORMAT_RGB8.get()->name);
  formats.push_back(colorspaces::ImageRGB8::FORMAT_RGB8_Z.get()->name);

  return (formats);
}

std::string CameraHandler::startCameraStreaming(const Ice::Current&){
  return (std::string("Nothing to do"));
}

void CameraHandler::stopCameraStreaming(const Ice::Current&) {
}

void CameraHandler::reset(const Ice::Current&)
{
}



} /* namespace jderobot */
