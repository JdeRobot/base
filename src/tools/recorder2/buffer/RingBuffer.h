/*
 *  Copyright (C) 2014 JdeRobot developers
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
 *  Authors : Roberto Calvo Palomino <rocapal [at] gsyc [dot] urjc [dot] es>
 *
 */
#include <boost/thread/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <logger/Logger.h>
#include <boost/lexical_cast.hpp>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

namespace recorder
{
    template<typename RingNode>
    class RingBuffer
    {
    public:
        struct pthread_create_args{
            std::vector<RingNode> buffer;
            std::string logName;
        };


        RingBuffer(long int maxTime){
            mMaxBufferTime = maxTime;
        }
        ~RingBuffer(){
            for (auto it = mBuffer.begin(); it < mBuffer.end(); it++ )
            {
                it->frame.release();
            }

            mBuffer.clear();
        }

        bool addNode(RingNode node){
            mBuffer.push_back(node);
            return checkBuffer();
        }
        void write(std::string nameLog, std::vector<int> compression){
            mCompression = compression;
            mNameLog = nameLog;

            mWriteBuffer.resize(mBuffer.size());
            std::copy( mBuffer.begin(), mBuffer.end(), mWriteBuffer.begin() );

            //std::cout << &(mBuffer[0].frame) << std::endl;
            //std::cout << &(mWriteBuffer[0].frame) << std::endl;

            pthread_attr_init(&mAttr);
            pthread_attr_setdetachstate(&mAttr, PTHREAD_CREATE_JOINABLE);

            pthread_create_args args;
            args.buffer= this->mWriteBuffer;
            args.logName=nameLog;

            pthread_create(&mThread, &mAttr, write_thread, &args);
        }


    private:

        bool checkBuffer(){

            for (auto it = mBuffer.begin(); it < mBuffer.end(); it++ )
            {
                auto newer = mBuffer.end()-1;
                //jderobot::Logger::getInstance()->info("Current: " + boost::lexical_cast<std::string>(it->relativeTime) +
                //		"  -  Newer: " + boost::lexical_cast<std::string>(newer->relativeTime));


                if ( ( newer->relativeTime - it->relativeTime) > mMaxBufferTime )
                {
                    it->frame.release();
                    mBuffer.erase(it);
                    return true;
                }
                else
                {
                    //jderobot::Logger::getInstance()->info("Older: " + boost::lexical_cast<std::string>(mBuffer.begin()->relativeTime) +
                    //			"  -  Newer: " + boost::lexical_cast<std::string>((mBuffer.end()-1)->relativeTime));

                    return false;
                }
            }

            return true;
        }

        long int mMaxBufferTime;
        std::vector<RingNode> mBuffer;
        std::vector<RingNode> mWriteBuffer;
        std::vector<int> mCompression;

        pthread_t mThread;
        pthread_attr_t mAttr;

        std::string mNameLog;

        static void *write_thread(void* context){
            pthread_create_args *args = (struct pthread_create_args *)context;

            RingNode::write(args->logName, args->buffer);
            pthread_exit(NULL);
            return NULL;
        }

    };


}