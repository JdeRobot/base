/*
*  Copyright (C) 1997-2017 JDERobot Developers Team
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
 *   Author : Francisco Miguel Rivas Montero <franciscomiguel.rivas@urjc.es>
 *
 */

#ifndef JDEROBOT_RGBDPOOL_H
#define JDEROBOT_RGBDPOOL_H

#include <rgbd.h>
#include <buffer/RGBDRingNode.h>
#include <buffer/RingBuffer.h>
#include "RecorderPool.h"
#include "PoolPaths.h"
#include "RecorderRGBD.h"

namespace recorder {

    class PoolWriteRGBD: public RecorderPool, public PoolPaths {
    public:
        PoolWriteRGBD(Ice::ObjectPrx prx, int freq, int poolSize, int cameraID, std::string imageFormat,
                       std::string fileFormat, std::vector<int> compression_params, const std::string& baseLogPath,
                       MODE mode, int bufferSeconds, std::string videoMode);
        virtual ~PoolWriteRGBD();
        void consumer_thread();
        void producer_thread();

        bool startCustomLog(const std::string& name, int seconds);
//        bool startCustomVideo(const std::string&  path, std::string name, int seconds);

        virtual void* consumer_thread_imp();
        virtual void* producer_thread_imp();


    private:
        jderobot::rgbdPrx rgbdPrx;
        std::vector<RecorderRGBDPtr> data;
        MODE mMode;
        int mLastSecondsLog;
        int mBufferSeconds;
        recorder::RingBuffer<RingBufferNS::RGBDRingNode>* mBuffer;
        pthread_mutex_t mModeMutex;


        std::vector<int> compression_params;
        std::string mNameLog;

        std::string mVideoMode;
        std::string fileFormat;


        boost::posix_time::ptime mFinalInit, mFinalEnd;






    };

    typedef boost::shared_ptr<PoolWriteRGBD> PoolWriteRGBDPtr;
}

#endif //JDEROBOT_RGBDPOOL_H
