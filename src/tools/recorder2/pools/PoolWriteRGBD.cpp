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

#include "PoolWriteRGBD.h"
#include <logger/Logger.h>
#include <boost/filesystem.hpp>


namespace recorder {

    PoolWriteRGBD::PoolWriteRGBD(Ice::ObjectPrx prx, int freq, int poolSize, int cameraID,  std::string imageFormat, std::string fileFormat,
                                     std::vector<int> compression_params,const std::string& baseLogPath, MODE mode, int bufferSeconds, std::string videoMode):
            RecorderPool(freq,poolSize,cameraID),
            PoolPaths(baseLogPath),
            mMode(mode),
            mLastSecondsLog(5),
            mBufferSeconds(bufferSeconds),
            mBuffer(NULL),
            fileFormat(fileFormat)

{
        this->rgbdPrx = jderobot::rgbdPrx::checkedCast(prx);
        if (0== this->rgbdPrx) {
            LOG(ERROR) << "Invalid proxy to RGBD";
        }
        this->compression_params=compression_params;



        mNameLog = "alarm1";
        mVideoMode = videoMode;
        if (mMode == SAVE_BUFFER)
        {
            LOG(INFO) << "Recorder run as buffer mode, with a buffer = " + boost::lexical_cast<std::string>(mBufferSeconds) + " seconds.";
            mBuffer = new RingBuffer<RingBufferNS::RGBDRingNode>(mBufferSeconds*1000,"./",RGBD);
        }
        else {
            createDevicePath(RGBD, cameraID);
            this->setLogFile(getDeviceLogFilePath(RGBD, cameraID));
        }
//        mCamType = this->cameraPrx->getImageFormat().at(0);
    }

    PoolWriteRGBD::~PoolWriteRGBD() {

    }

    void PoolWriteRGBD::consumer_thread() {
        pthread_mutex_lock(&(this->mutex));
        if (this->data.size() > 0) {
            RecorderRGBDPtr data2Save;

            data2Save=RecorderRGBDPtr(new RecorderRGBD(*this->data[0]));
            this->data.erase(this->data.begin());

            long long int relative;
            relative = *(this->its.begin());
            this->its.erase(this->its.begin());
            pthread_mutex_unlock(&(this->mutex));

            std::stringstream imageFileName;//create a stringstream
            imageFileName << getDeviceLogPath(RGBD, this->deviceID) << relative ; //<< "." << fileFormat;

            MODE currentMode;
            pthread_mutex_lock(&(this->mModeMutex));
            currentMode = mMode;
            pthread_mutex_unlock(&(this->mModeMutex));

            // Normal mode
            if (currentMode == WRITE_FRAME) {

                cv::imwrite(imageFileName.str() + "-depth." + fileFormat, data2Save->depth, this->compression_params);
                cv::imwrite(imageFileName.str() + "-rgb." + fileFormat, data2Save->rgb, this->compression_params);

                this->logfile << relative << std::endl;

            } else {
                RingBufferNS::RGBDRingNode node;
                // Save buffer in memory mode == SAVE_BUFFER
                cv::Mat saveImage;

//                if (mVideoMode.compare("Video") == 0) {
//                    if (mCamType.compare("RGB8") == 0) {
//                        saveImage.create(img2Save.size(), CV_8UC3);
//                        cv::cvtColor(img2Save, saveImage, CV_BGR2RGB);
//                    } else if (mCamType.compare("DEPTH8_16") == 0) {
//                        saveImage.create(img2Save.size(), CV_8UC3);
//                        std::vector<cv::Mat> layers;
//                        cv::split(img2Save, layers);
//                        cv::cvtColor(layers[0], saveImage, CV_GRAY2RGB);
//
//                    } else {
//
//                        LOG(WARNING) << "mCamType not recognized " + mCamType;
//                    }
//                } else
                if (mVideoMode.compare("Log") == 0) {
                    node.rgb=data2Save->rgb.clone();
                    node.depth=data2Save->depth.clone();
                }



                node.cameraId = deviceID;
                node.relativeTime = relative;

                mBuffer->addNode(node);

                if (currentMode == WRITE_BUFFER) {
                    std::string fileLogPath = getCustomLogFilePath(RGBD, this->deviceID, mNameLog);
                    this->setLogFile(fileLogPath);


                    LOG(INFO) << "Init recording log: " + mNameLog + " (camera" +
                                 boost::lexical_cast<std::string>(this->deviceID) + " ) with "
                                 + boost::lexical_cast<std::string>(mBufferSeconds)
                                 + " buffer seconds and " + boost::lexical_cast<std::string>(mLastSecondsLog) +
                                 " at the end!";

                    mBuffer->write(mNameLog, this->compression_params);

                    pthread_mutex_lock(&(this->mModeMutex));
                    mMode = WRITE_END_LOG;
                    pthread_mutex_unlock(&(this->mModeMutex));

                    mFinalInit = boost::posix_time::second_clock::local_time();

                }
                    // Save the final seconds of recording and save 'data' file
                else if (currentMode == WRITE_END_LOG) {

                    mFinalEnd = boost::posix_time::second_clock::local_time();
                    boost::posix_time::time_duration total = mFinalEnd - mFinalInit;

                    if (total.seconds() > mLastSecondsLog) {
                        std::vector<int> res;
                        std::string dataPath = getCustomLogPath(IMAGES, this->deviceID, mNameLog);
                        boost::filesystem::directory_iterator end_itr;
                        for (boost::filesystem::directory_iterator itr(dataPath); itr != end_itr; ++itr) {
                            if (itr->path().generic_string().find("png") == std::string::npos)
                                continue;

                            unsigned begin = itr->path().generic_string().find_last_of("/") + 1;
                            unsigned end = itr->path().generic_string().find_last_of(".");
                            if (begin == std::string::npos || end == std::string::npos) {
                                LOG(WARNING) << "Error while parsed file " + itr->path().generic_string();
                                continue;
                            }

                            res.push_back(atoi(itr->path().generic_string().substr(begin, end - begin).c_str()));
                        }

                        std::sort(res.begin(), res.end());
                        for (std::vector<int>::iterator it = res.begin(); it < res.end(); it++) {
                            this->logfile << *it << std::endl;
                        }

                        this->logfile.close();
                        LOG(INFO) << "End recording log: " + mNameLog;

                        // Save the videoa
//                        if (mVideoMode.compare("Video") == 0) {
//                            std::stringstream fileData, fileImage, command, command_video, fileVideo, tmpFileVideo;
//
//                            std::string basePath = getCustomLogPath(RGBD, this->deviceID, mNameLog);
//                            fileData << basePath << "cameraData.jde";
//                            fileImage << basePath << "list.txt";
//
//                            command << "cat " << fileData.str() << " | sed -e 's/$/.png/g' > " << fileImage.str();
//
//                            system(command.str().c_str());
//
//                            fileVideo << mNamePathVideo << "/" << mNameLog << "-" << this->deviceID << "-" << mCamType
//                                      << ".avi";
//                            tmpFileVideo << mNamePathVideo << "/" << mNameLog << "-" << this->deviceID << "-"
//                                         << mCamType << ".mp4";
//
//                            command_video << "cd " << basePath << ";";
//                            command_video
//                                    << "mencoder mf://@list.txt -mf w=320:h=240:fps=10:type=png -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell -oac copy -o "
//                                    << tmpFileVideo.str() << ";";
//                            command_video << "ffmpeg -y -i " << tmpFileVideo.str() << " -vcodec libx264 "
//                                          << fileVideo.str() << ";";
//                            command_video << "rm -f " << tmpFileVideo.str() << ";";
//                            command_video << "cd -; rm -rf " << "data-" << mNameLog;
//
//                            std::cout << command_video.str() << std::endl;
//
//                            system(command_video.str().c_str());
//                        }

                        pthread_mutex_lock(&(this->mModeMutex));
                        mMode = SAVE_BUFFER;
                        pthread_mutex_unlock(&(this->mModeMutex));
                    } else {
                        std::string basePath = getCustomLogPath(RGBD, this->deviceID, mNameLog);
                        std::stringstream path;
                        path << basePath << relative;
                        cv::imwrite(path.str() + "-depth." + fileFormat, data2Save->depth, this->compression_params);
                        cv::imwrite(path.str() + "-rgb." + fileFormat, data2Save->rgb, this->compression_params);
                    }
                }
            }

        } else
            pthread_mutex_unlock(&(this->mutex));
        usleep(1000);

    }

    void PoolWriteRGBD::producer_thread() {
        jderobot::rgbData dataInput = this->rgbdPrx->getData();


        RecorderRGBDPtr dataPtr = RecorderRGBDPtr ( new RecorderRGBD(dataInput));
        struct timeval now;
        gettimeofday(&now,NULL);
        long long int relative;
        relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
        pthread_mutex_lock(&(this->mutex));
        while (this->data.size() > this->poolSize){
            pthread_mutex_unlock(&(this->mutex));
            usleep(100);
            pthread_mutex_lock(&(this->mutex));
        }
        this->data.push_back(dataPtr);
        this->its.push_back(relative);
        pthread_mutex_unlock(&(this->mutex));
        gettimeofday(&now,NULL);

        long long int totalNow=now.tv_sec*1000000+now.tv_usec;
        long long int totalLast=lastTime.tv_sec*1000000+lastTime.tv_usec;

        float sleepTime =this->cycle - (totalNow-totalLast)/1000.;

        if(sleepTime < 0 )
            sleepTime = 0;
        usleep(sleepTime*1000);
        gettimeofday(&lastTime,NULL);

    }

    void *PoolWriteRGBD::consumer_thread_imp() {
        while (this->getActive()){
            if (this->getRecording())
                this->consumer_thread();
            else
                usleep(1000);
        }

        pthread_exit(NULL);
        return NULL;    }

    void *PoolWriteRGBD::producer_thread_imp() {

        while (this->getActive()){
            if (this->getRecording())
                this->producer_thread();
            else
                usleep(1000);
        }
        pthread_exit(NULL);    }

    bool PoolWriteRGBD::startCustomLog (const std::string& name, int seconds)
    {
        bool ret;

        pthread_mutex_lock(&(this->mModeMutex));

        if (mMode != SAVE_BUFFER)
        {
            LOG(WARNING) << "Can't handle recording '" + name + "' because there is a recording active!" ;
            ret = false;
        }
        else
        {
            mNameLog = name;
            mLastSecondsLog = seconds;
            mMode = WRITE_BUFFER;

            ret = true;
        }
        pthread_mutex_unlock(&(this->mModeMutex));

        return ret;
    }

}