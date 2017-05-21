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


namespace recorder {

    PoolWriteRGBD::PoolWriteRGBD(Ice::ObjectPrx prx, int freq, int poolSize, int cameraID,  std::string imageFormat, std::string fileFormat,
                                     std::vector<int> compression_params,const std::string& baseLogPath, MODE mode, int bufferSeconds, std::string videoMode):
            RecorderPool(freq,poolSize,cameraID),
            PoolPaths(baseLogPath),
//            mBuffer(NULL),
//            mLastSecondsLog(5),
//            mBufferSeconds(bufferSeconds),
            mMode(mode)
    {
//        this->cameraPrx = jderobot::CameraPrx::checkedCast(prx);
//        if (0== this->cameraPrx) {
//            LOG(ERROR) << "Invalid proxy";
//        }
//        this->compression_params=compression_params;
//        this->imageFormat=imageFormat;
//        this->fileFormat=fileFormat;
//
//
//        mNameLog = "alarm1";
//        mVideoMode = videoMode;
//        if (mMode == SAVE_BUFFER)
//        {
//            LOG(INFO) << "Recorder run as buffer mode, with a buffer = " + boost::lexical_cast<std::string>(mBufferSeconds) + " seconds.";
//            mBuffer = new RingBuffer(mBufferSeconds*1000);
//        }
//        else {
//            createDevicePath(IMAGES, cameraID);
//            this->setLogFile(getDeviceLogFilePath(IMAGES, cameraID));
//        }
//        mCamType = this->cameraPrx->getImageFormat().at(0);
    }

    PoolWriteRGBD::~PoolWriteRGBD() {

    }

    void PoolWriteRGBD::consumer_thread() {
        pthread_mutex_lock(&(this->mutex));
        if (this->data.size() > 0) {
//            RecorderRGBDPtr data2Save;
//
//            data2Save=RecorderRGBDPtr(new RecorderRGBD(this->data[0]));
//            this->data.erase(this->data.begin());
//
//            long long int relative;
//            relative = *(this->its.begin());
//            this->its.erase(this->its.begin());
//            pthread_mutex_unlock(&(this->mutex));
//
//            std::stringstream imageFileName;//create a stringstream
//            imageFileName << getDeviceLogPath(IMAGES, this->deviceID) << relative << "." << fileFormat;
//
//            MODE currentMode;
//            pthread_mutex_lock(&(this->mModeMutex));
//            currentMode = mMode;
//            pthread_mutex_unlock(&(this->mModeMutex));
//
//            // Normal mode
//            if (currentMode == WRITE_FRAME) {
//
//                cv::imwrite(imageFileName.str(), img2Save, this->compression_params);
//                this->logfile << relative << std::endl;
//
//            } else {
//                // Save buffer in memory mode == SAVE_BUFFER
//                cv::Mat saveImage;
//
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
//                } else if (mVideoMode.compare("Log") == 0) {
//                    saveImage.create(img2Save.size(), img2Save.type());
//                    img2Save.copyTo(saveImage);
//                }
//
//
//                RingBuffer::RingNode node;
//                node.cameraId = deviceID;
//                node.relativeTime = relative;
//
//                saveImage.copyTo(node.frame);
//                mBuffer->addNode(node);
//
//                if (currentMode == WRITE_BUFFER) {
//                    std::string fileLogPath = getCustomLogFilePath(IMAGES, this->deviceID, mNameLog);
//                    this->setLogFile(fileLogPath);
//
//
//                    LOG(INFO) << "Init recording log: " + mNameLog + " (camera" +
//                                 boost::lexical_cast<std::string>(this->deviceID) + " ) with "
//                                 + boost::lexical_cast<std::string>(mBufferSeconds)
//                                 + " buffer seconds and " + boost::lexical_cast<std::string>(mLastSecondsLog) +
//                                 " at the end!";
//
//                    mBuffer->write(mNameLog, this->compression_params);
//
//                    pthread_mutex_lock(&(this->mModeMutex));
//                    mMode = WRITE_END_LOG;
//                    pthread_mutex_unlock(&(this->mModeMutex));
//
//                    mFinalInit = boost::posix_time::second_clock::local_time();
//
//                }
//                    // Save the final seconds of recording and save 'data' file
//                else if (currentMode == WRITE_END_LOG) {
//
//                    mFinalEnd = boost::posix_time::second_clock::local_time();
//                    boost::posix_time::time_duration total = mFinalEnd - mFinalInit;
//
//                    if (total.seconds() > mLastSecondsLog) {
//                        std::vector<int> res;
//                        std::string dataPath = getCustomLogPath(IMAGES, this->deviceID, mNameLog);
//                        boost::filesystem::directory_iterator end_itr;
//                        for (boost::filesystem::directory_iterator itr(dataPath); itr != end_itr; ++itr) {
//                            if (itr->path().generic_string().find("png") == std::string::npos)
//                                continue;
//
//                            unsigned begin = itr->path().generic_string().find_last_of("/") + 1;
//                            unsigned end = itr->path().generic_string().find_last_of(".");
//                            if (begin == std::string::npos || end == std::string::npos) {
//                                LOG(WARNING) << "Error while parsed file " + itr->path().generic_string();
//                                continue;
//                            }
//
//                            res.push_back(atoi(itr->path().generic_string().substr(begin, end - begin).c_str()));
//                        }
//
//                        std::sort(res.begin(), res.end());
//                        for (std::vector<int>::iterator it = res.begin(); it < res.end(); it++) {
//                            this->logfile << *it << std::endl;
//                        }
//
//                        this->logfile.close();
//                        LOG(INFO) << "End recording log: " + mNameLog;
//
//                        // Save the videoa
//                        if (mVideoMode.compare("Video") == 0) {
//                            std::stringstream fileData, fileImage, command, command_video, fileVideo, tmpFileVideo;
//
//                            std::string basePath = getCustomLogPath(IMAGES, this->deviceID, mNameLog);
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
//
//                        pthread_mutex_lock(&(this->mModeMutex));
//                        mMode = SAVE_BUFFER;
//                        pthread_mutex_unlock(&(this->mModeMutex));
//                    } else {
//                        std::string basePath = getCustomLogPath(IMAGES, this->deviceID, mNameLog);
//                        std::stringstream path;
//                        path << basePath << relative << "." << fileFormat;
//                        cv::imwrite(path.str(), saveImage, this->compression_params);
//                    }
//                }
//            }

        } else
            pthread_mutex_unlock(&(this->mutex));
        usleep(1000);

    }

    void PoolWriteRGBD::producer_thread() {

    }

    void *PoolWriteRGBD::consumer_thread_imp() {
        return nullptr;
    }

    void *PoolWriteRGBD::producer_thread_imp() {
        return nullptr;
    }

}