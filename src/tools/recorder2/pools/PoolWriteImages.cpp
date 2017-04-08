/*
 * poolWriteImages.cpp
 *
 *  Created on: 03/05/2013
 *      Author: frivas
 */

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "PoolWriteImages.h"
#include <logger/Logger.h>

namespace recorder{
poolWriteImages::poolWriteImages(Ice::ObjectPrx prx, int freq, int poolSize, int cameraID,  std::string imageFormat, std::string fileFormat, std::vector<int> compression_params):RecorderPool(freq,poolSize,cameraID) {
	// TODO Auto-generated constructor stub
    this->cameraPrx = jderobot::CameraPrx::checkedCast(prx);
    if (0== this->cameraPrx) {
        LOG(ERROR) << "Invalid proxy";
    }
	this->compression_params=compression_params;
	this->imageFormat=imageFormat;
    this->fileFormat=fileFormat;
    this->cycle = 1000.0/freq;
	gettimeofday(&lastTime,NULL);
}

poolWriteImages::~poolWriteImages() {
	this->logfile.close();
	// TODO Auto-generated destructor stub
}

    void poolWriteImages::consumer_thread(){

        pthread_mutex_lock(&(this->mutex));
        if (this->images.size()>0){
            cv::Mat img2Save;

            this->images.begin()->copyTo(img2Save);
            this->images.erase(this->images.begin());

            long long int relative;
            relative=*(this->its.begin());
            this->its.erase(this->its.begin());
            pthread_mutex_unlock(&(this->mutex));

            std::stringstream buff;//create a stringstream
            buff << "data/images/camera" << deviceID << "/" << relative << "." << fileFormat;


            std::stringstream dataPath;
            dataPath << "data-" << mNameLog << "/images/camera" << this->deviceID << "/";

            MODE currentMode;
            pthread_mutex_lock(&(this->mModeMutex));
            currentMode = mMode;
            pthread_mutex_unlock(&(this->mModeMutex));

            // Normal mode
            if (currentMode == WRITE_FRAME)
            {

                cv::imwrite(buff.str(), img2Save,this->compression_params);
                this->logfile << relative<< std::endl;

            }
            else
            {
                // Save buffer in memory mode == SAVE_BUFFER
                cv::Mat saveImage;

                if (mVideoMode.compare("Video")==0)
                {
                    if (mCamType.compare("RGB8")==0)
                    {
                        saveImage.create(img2Save.size(), CV_8UC3);
                        cv::cvtColor(img2Save,saveImage,CV_BGR2RGB);
                    }
                    else if (mCamType.compare("DEPTH8_16")==0)
                    {
                        saveImage.create(img2Save.size(), CV_8UC3);
                        std::vector<cv::Mat> layers;
                        cv::split(img2Save, layers);
                        cv::cvtColor(layers[0],saveImage,CV_GRAY2RGB);

                    }
                    else
                    {

                        LOG(WARNING) << "mCamType not recognized " + mCamType;
                    }
                }
                else if (mVideoMode.compare("Log")==0)
                {
                    saveImage.create(img2Save.size(), img2Save.type());
                    img2Save.copyTo(saveImage);
                }


                RingBuffer::RingNode node;
                node.cameraId = deviceID;
                node.relativeTime = relative;

                saveImage.copyTo(node.frame);
                mBuffer->addNode(node);

                if (currentMode == WRITE_BUFFER)
                {

                    //Create dir
                    boost::filesystem::path dir(dataPath.str());
                    boost::filesystem::create_directories(dir);

                    std::stringstream filePath;
                    filePath << "data-" << mNameLog << "/images/camera" << this->deviceID << "/cameraData.jde";
                    this->logfile.open(filePath.str().c_str());


                    LOG(INFO) << "Init recording log: " + mNameLog + " (camera" + boost::lexical_cast<std::string>(this->deviceID) +  " ) with "
                                                          + boost::lexical_cast<std::string>(mBufferSeconds)
                                                          + " buffer seconds and " + boost::lexical_cast<std::string>(mLastSecondsLog) + " at the end!" ;

                    mBuffer->write(mNameLog, this->compression_params);

                    pthread_mutex_lock(&(this->mModeMutex));
                    mMode = WRITE_END_LOG;
                    pthread_mutex_unlock(&(this->mModeMutex));

                    mFinalInit = boost::posix_time::second_clock::local_time();

                }
                    // Save the final seconds of recording and save 'data' file
                else if (currentMode == WRITE_END_LOG)
                {

                    mFinalEnd = boost::posix_time::second_clock::local_time();
                    boost::posix_time::time_duration total = mFinalEnd - mFinalInit;

                    if (total.seconds() > mLastSecondsLog)
                    {
                        std::vector<int> res;

                        boost::filesystem::directory_iterator end_itr;
                        for ( boost::filesystem::directory_iterator itr( dataPath.str() ); itr != end_itr; ++itr )
                        {
                            if ( itr->path().generic_string().find("png") == std::string::npos )
                                continue;

                            unsigned begin = itr->path().generic_string().find_last_of("/") + 1;
                            unsigned end = itr->path().generic_string().find_last_of(".");
                            if (begin == std::string::npos || end == std::string::npos)
                            {
                                LOG(WARNING) << "Error while parsed file " + itr->path().generic_string();
                                continue;
                            }

                            res.push_back( atoi(itr->path().generic_string().substr(begin, end-begin).c_str()) );
                        }

                        std::sort(res.begin(),res.end());
                        for (std::vector<int>::iterator it = res.begin(); it < res.end(); it++)
                        {
                            this->logfile << *it << std::endl;
                        }

                        this->logfile.close();
                        LOG(INFO) << "End recording log: " + mNameLog ;

                        // Save the video
                        if (mVideoMode.compare("Video")==0)
                        {
                            std::stringstream basePath, fileData, fileImage, command, command_video, fileVideo, tmpFileVideo;
                            basePath << "data-" << mNameLog << "/images/camera" << this->deviceID << "/";
                            fileData << basePath.str() << "cameraData.jde";
                            fileImage << basePath.str() << "list.txt";

                            command << "cat " << fileData.str() << " | sed -e 's/$/.png/g' > " << fileImage.str();

                            system(command.str().c_str());

                            fileVideo << mNamePathVideo << "/" << mNameLog << "-" << this->deviceID << "-" << mCamType << ".avi";
                            tmpFileVideo << mNamePathVideo << "/" << mNameLog << "-" << this->deviceID << "-" << mCamType << ".mp4";

                            command_video << "cd " << basePath.str() << ";" ;
                            command_video << "mencoder mf://@list.txt -mf w=320:h=240:fps=10:type=png -ovc lavc -lavcopts vcodec=mpeg4:mbd=2:trell -oac copy -o " << tmpFileVideo.str() << ";" ;
                            command_video << "ffmpeg -y -i " << tmpFileVideo.str() << " -vcodec libx264 " << fileVideo.str() << ";" ;
                            command_video << "rm -f " << tmpFileVideo.str() << ";" ;
                            command_video << "cd -; rm -rf " << "data-" << mNameLog;

                            std::cout << command_video.str() << std::endl;

                            system(command_video.str().c_str());
                        }

                        pthread_mutex_lock(&(this->mModeMutex));
                        mMode = SAVE_BUFFER;
                        pthread_mutex_unlock(&(this->mModeMutex));
                    }
                    else
                    {
                        std::stringstream path;
                        path << "data-" << mNameLog << "/images/camera" << deviceID << "/" << relative << "." << fileFormat;
                        cv::imwrite(path.str(), saveImage ,this->compression_params);
                    }
                }
            }

        }
        else
            pthread_mutex_unlock(&(this->mutex));
        usleep(1000);

    }

void poolWriteImages::producer_thread(){
    jderobot::ImageDataPtr imageData=this->cameraPrx->getImageData(this->imageFormat);
    colorspaces::Image::FormatPtr fmt = colorspaces::Image::Format::searchFormat(imageData->description->format);
    cv::Mat image;
    image.create(cv::Size(imageData->description->width, imageData->description->height), CV_8UC3);
    memcpy((unsigned char *) image.data ,&(imageData->pixelData[0]), image.cols*image.rows * 3);
    struct timeval now;
    gettimeofday(&now,NULL);
    long long int relative;
    relative=((now.tv_sec*1000000+now.tv_usec) - (syncInitialTime.tv_sec*1000000+syncInitialTime.tv_usec))/1000;
    pthread_mutex_lock(&(this->mutex));
    while (this->images.size() > this->poolSize){
        pthread_mutex_unlock(&(this->mutex));
        usleep(100);
        pthread_mutex_lock(&(this->mutex));
    }
    this->images.push_back(image);
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

	bool poolWriteImages::startCustomLog (std::string name, int seconds)
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

	bool poolWriteImages::startCustomVideo(std::string path, std::string name, int seconds)
	{
		mNamePathVideo = path;
		return startCustomLog(name, seconds);
	}


    void* poolWriteImages::producer_thread_imp(){
        while (this->getActive()){
            if (this->getRecording())
                this->producer_thread();
            else
                usleep(1000);
        }
        pthread_exit(NULL);
    }

    void* poolWriteImages::consumer_thread_imp(){
        while (this->getActive()){
            if (this->getRecording())
                this->consumer_thread();
            else
                usleep(1000);
        }

        pthread_exit(NULL);
        return NULL;
    }


} //namespace

