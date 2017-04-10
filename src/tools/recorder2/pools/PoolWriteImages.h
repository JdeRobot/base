/*
*  Copyright (C) 1997-2013 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 *   Authors : Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>,
 *             Jose María Cañas Plaza <jmplaza@gsyc.es>
 *
 */
#ifndef POOLWRITEIMAGES_H_
#define POOLWRITEIMAGES_H_

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <time.h>
#include <jderobot/camera.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include <fstream>
#include <buffer/RingBuffer.h>
#include "RecorderPool.h"
#include "PoolPaths.h"


namespace recorder{


class poolWriteImages: public RecorderPool, public PoolPaths {
public:
	enum MODE
	{
		WRITE_FRAME = 0,
		SAVE_BUFFER,
		WRITE_BUFFER,
		WRITE_END_LOG
	};


    poolWriteImages(Ice::ObjectPrx prx, int freq, int poolSize, int cameraID, std::string imageFormat,
                    std::string fileFormat, std::vector<int> compression_params, const std::string& baseLogPath,
                    MODE mode, int bufferSeconds, std::string videoMode
    );
	virtual ~poolWriteImages();
	void consumer_thread();
	void producer_thread();

	bool startCustomLog(std::string name, int seconds);
	bool startCustomVideo(std::string path, std::string name, int seconds);

	virtual void* consumer_thread_imp();
	virtual void* producer_thread_imp();


private:
	std::vector<cv::Mat> images;
	std::vector<int> compression_params;
	std::string imageFormat;
    std::string fileFormat;
	jderobot::CameraPrx cameraPrx;

	// write log by demand
	recorder::RingBuffer* mBuffer;
	pthread_mutex_t mModeMutex;
	std::string mNameLog;
	int mLastSecondsLog;
	int mBufferSeconds;

	MODE mMode;
	std::string mNamePathVideo;
	boost::posix_time::ptime mFinalInit, mFinalEnd;
	std::string mVideoMode;
	std::string mCamType;
	//threads

};
	typedef boost::shared_ptr<poolWriteImages> poolWriteImagesPtr;
} //NAMESPACE

#endif /* POOLWRITEIMAGES_H_ */
