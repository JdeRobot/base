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
}

void RunTimeGui::emitRunningStateById(int id) {
    std::stringstream strstream;
    strstream << "emitRunningStateById " << id;
    msgQueue.push(strstream.str());
}

void RunTimeGui::emitLoadFromRoot() {
    std::stringstream strstream;
    strstream << "emitLoadFromRoot";
    msgQueue.push(strstream.str());
}

void RunTimeGui::emitActiveStateById(int id) {
    std::stringstream strstream;
    strstream << "emitActiveStateById " << id;
    msgQueue.push(strstream.str());
}

void* RunTimeGui::loopStaticIPC(void* owner) {
    ((RunTimeGui*)owner)->loopIPC();
}

void RunTimeGui::loopIPC() {
    while(true) {
        if (msgQueue.size() > 0) {
            std::cout << "loop ipc data:" << ipcData << ":" << std::endl;
            if (ipcData[0] == '0') {
                std::cout << "write data" << std::endl;
                std::string msg = msgQueue.front();
                msgQueue.pop();
                strncpy(ipcData, msg.c_str(), 1024);
                std::cout << "written data:" << ipcData << ":" << std::endl;
            }
        }

        usleep(1000);
    }
}

void RunTimeGui::createSharedMem() {
    key_t fkey;
    int shmid;
    int mode;

//    if ((fkey = ftok("/tmp/visualstates", 'R')) == -1) {
//        std::cerr << "cannot read shared mem /tmp/visualstates" << std::endl;
//        return;
//    }

    if ((shmid = shmget(123456, 0, 0644)) == -1) {
        std::cerr << "error 2" << std::endl;
        return;
    }

    ipcData = (char*) shmat(shmid, (void *)0, 0);
    if (ipcData == (char *)(-1)) {
        std::cerr << "exit" << std::endl;
        return;
    }
}
