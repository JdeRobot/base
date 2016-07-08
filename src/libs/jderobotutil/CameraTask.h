/*
 * CameraTask.h
 *
 *  Created on: 29/6/2015
 *      Author: frivas
 */

#ifndef SRC_STABLE_LIBS_JDEROBOTUTIL_CMAKEFILES_CAMERATASK_H_
#define SRC_STABLE_LIBS_JDEROBOTUTIL_CMAKEFILES_CAMERATASK_H_

#include <IceUtil/Thread.h>
#include <jderobot/camera.h>
#include <list>
#include <opencv2/core/core.hpp>

namespace jderobot {

class CameraTask: public IceUtil::Thread {
 public:
  CameraTask(const jderobot::Camera* camera, int fps);
  virtual ~CameraTask();

  void pushJob(const jderobot::AMD_ImageProvider_getImageDataPtr& cb, std::string format);
  void print_md5_sum(unsigned char* md);
  virtual void run();

  //to implement
  virtual void createCustomImage(cv::Mat& image) =0;

 private:
    void sendImage(jderobot::AMD_ImageProvider_getImageDataPtr cb, std::string& format, const cv::Mat& image);

  jderobot::ImageDescriptionPtr imageDescription;
  jderobot::CameraDescriptionPtr cameraDescription;
  const jderobot::Camera* mycamera;
  IceUtil::Mutex requestsMutex;
  std::list<std::pair<jderobot::AMD_ImageProvider_getImageDataPtr, std::string>> requests;
  int fps;
  bool _done;
  jderobot::ImageDataPtr reply;
};

} /* namespace jderobot */

#endif /* SRC_STABLE_LIBS_JDEROBOTUTIL_CMAKEFILES_CAMERATASK_H_ */
