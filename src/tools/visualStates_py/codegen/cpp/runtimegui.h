#ifndef RUNTIMEGUI_H
#define RUNTIMEGUI_H

#include <queue>
#include <pthread.h>
#include <string>

class RunTimeGui {
protected:
    pthread_t threadIPC;
    std::queue<std::string> msgQueue;
    char* ipcData;

    void createSharedMem();

public:
    RunTimeGui();

    void emitRunningStateById(int id);
    void emitLoadFromRoot();
    void emitActiveStateById(int id);

    void addState(int id, std::string name, bool intial, float x, float y, int parentId);

    static void* loopStaticIPC(void*);
    void loopIPC();
};


#endif
