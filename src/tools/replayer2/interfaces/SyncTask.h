//
// Created by frivas on 28/04/17.
//

#ifndef JDEROBOT_SYNCTASK_H
#define JDEROBOT_SYNCTASK_H


#include <IceUtil/Thread.h>
#include <fstream>
#include <logger/Logger.h>

namespace  replayer {

    template<typename InterfaceType>
    class SyncTask : public IceUtil::Thread {
    public:
        SyncTask(const std::string&  pathIn, const std::string& jdeFilePath,InterfaceType interface) {
            this->path = pathIn;
            this->jdeFilePath = jdeFilePath;
            this->onPause = false;
            this->interface=interface;
        }

        ~SyncTask() {
        }


        virtual void generateData()=0;

        virtual void run() {
            std::string fileName(this->path + this->jdeFilePath);
            std::ifstream myfile(fileName.c_str());
            if (!myfile.is_open())
                LOG(ERROR) << "Error while trying to open: " + fileName;
            while (this->isAlive()) {
                while (myfile.good()) {
                    bool playing = this->interface->syncController->getPlay();

                    this->onPause = !playing;
                    while (!playing) {
                        playing = this->interface->syncController->getPlay();
                        long long int pauseStatus = this->interface->syncController->getSyncTime();
                        if (pauseStatus != this->interface->initState) {
                            this->interface->initState = pauseStatus;
                            break;
                        }
                        usleep(10000);
                        continue;
                    }

                    if (this->onPause) {
                        this->interface->initState = this->interface->syncController->getSyncTime();
                        myfile.close();
                        myfile.open(fileName.c_str());
                    }

                    getline(myfile, lineData);
                    std::istringstream sTemp(lineData);
                    long long int relative;
                    sTemp >> relative;
                    IceUtil::Time pretime = IceUtil::Time::now();
                    long long int checkState = (pretime.toMicroSeconds()) / 1000;


                    while ((((relative) - (checkState - this->interface->initState)) < 0) && (myfile.good())) {
                        getline(myfile, lineData);
                        std::stringstream ssTemp(lineData);
                        std::string sTemp;
                        std::getline(ssTemp, sTemp, ' ');
                        std::istringstream usTemp(sTemp);
                        usTemp >> relative;
                    }
                    if (!myfile.good()) {
                        if (this->onPause)
                            continue;
                        else
                            break;
                    }



                    IceUtil::Time a = IceUtil::Time::now();
                    long long int actualState = (a.toMicroSeconds()) / 1000;
                    if ((actualState - this->interface->initState) < relative) {
                        usleep(((relative) - (actualState - this->interface->initState)) * 1000);
                    }
                    this->interface->dataMutex.lock();
                    generateData();
                    this->interface->dataMutex.unlock();

                }
                myfile.close();
                this->interface->initState = this->interface->syncController->wait();
                myfile.open(fileName.c_str());

            }

        }

    private:
        std::string jdeFilePath;
        bool onPause;
    protected:
        std::string lineData;
        std::string path;
        InterfaceType interface;


    };

}
#endif //JDEROBOT_SYNCTASK_H
