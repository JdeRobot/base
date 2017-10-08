/*
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Asik (asik.okan@gmail.com)

*/
#include "runtimegui.h"
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <sstream>
#include <iostream>
#include <unistd.h>

RunTimeGui::RunTimeGui() {
    // create shared memory
    createSharedMem();
    pthread_create(&threadIPC, NULL, &RunTimeGui::loopStaticIPC, this);
    ipcData[0] == '0';
}

void RunTimeGui::emitRunningStateById(int id) {
    std::stringstream strstream;
//    strstream << "emitRunningStateById " << id;
    msgQueue.push(strstream.str());
//    std::cout << "running state:" << id << std::endl;
}

void RunTimeGui::emitLoadFromRoot() {
    std::stringstream strstream;
//    strstream << "emitLoadFromRoot";
    msgQueue.push(strstream.str());
}

void RunTimeGui::emitActiveStateById(int id) {
    std::stringstream strstream;
//    strstream << "emitActiveStateById " << id;
    msgQueue.push(strstream.str());
}

void* RunTimeGui::loopStaticIPC(void* owner) {
    ((RunTimeGui*)owner)->loopIPC();
}

void RunTimeGui::loopIPC() {
    while(true) {
        if (msgQueue.size() > 0) {
            if (ipcData[0] == '0') {
                std::string msg = msgQueue.front();
                msgQueue.pop();
                strncpy(ipcData, msg.c_str(), 1024);
            }
        }

        usleep(1000);
    }
}

void RunTimeGui::createSharedMem() {
    key_t fkey;
    int shmid;
    int mode;
    int numTrial = 10;
    int trial = 0;

    while (trial < numTrial) {
        if ((shmid = shmget(123456, 0, 0644)) == -1) {
            std::cerr << "error shmget try for " << trial << std::endl;
        } else {
            break;
        }
        trial++;
        usleep(100000); // sleep for 100 milliseconds
    }

    if (trial < numTrial) {
//        std::cout << "shmget works" << std::endl;
    } else if (trial >= numTrial) {
        std::cerr << "we could not be able to get shared memory in " << numTrial << " trials." << std::endl;
        return;
    }

    ipcData = (char*) shmat(shmid, (void *)0, 0);
    if (ipcData == (char *)(-1)) {
        std::cerr << "error shmat" << std::endl;
        return;
    }
}
