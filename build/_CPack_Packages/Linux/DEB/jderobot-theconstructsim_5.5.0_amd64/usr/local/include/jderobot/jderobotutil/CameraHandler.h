/*
 * CameraHandler.h
 *
 *  Created on: 29/6/2015
 *      Author: frivas
 */

#ifndef SRC_STABLE_LIBS_JDEROBOTUTIL_CAMERAHANDLER_H_
#define SRC_STABLE_LIBS_JDEROBOTUTIL_CAMERAHANDLER_H_

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <IceStorm/IceStorm.h>
#include <jderobot/camera.h>
#include <jderobot/image.h>
#include <visionlib/colorspaces/colorspacesmm.h>

namespace jderobot {

class CameraHandler: virtual public jderobot::Camera {


public:

  CameraHandler(std::string propertyPrefix, Ice::CommunicatorPtr ic);

  std::string getName ();

  virtual ~CameraHandler();

  virtual jderobot::ImageDescriptionPtr getImageDescription(const Ice::Current& c);
  virtual jderobot::CameraDescriptionPtr getCameraDescription(const Ice::Current& c);
  virtual Ice::Int setCameraDescription(const jderobot::CameraDescriptionPtr &description, const Ice::Current& c);
  virtual jderobot::ImageFormat getImageFormat(const Ice::Current& c);
  virtual void getImageData_async(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, const std::string& format, const Ice::Current& c)=0;
  virtual std::string startCameraStreaming(const Ice::Current&);
  virtual void stopCameraStreaming(const Ice::Current&);
  virtual void reset(const Ice::Current&);

  Ice::PropertiesPtr prop;
  std::string prefix;
  colorspaces::Image::FormatPtr imageFmt;
  jderobot::ImageDescriptionPtr imageDescription;
  jderobot::CameraDescriptionPtr cameraDescription;
  jderobot::ImageConsumerPrx imageConsumer;
  int mirror;


private:






}; // end class CameraI
} /* namespace jderobot */

#endif /* SRC_STABLE_LIBS_JDEROBOTUTIL_CAMERAHANDLER_H_ */
