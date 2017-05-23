//
// Created by frivas on 30/04/17.
//

#ifndef JDEROBOT_ReplayControllerInterfaceNTERFACE_H
#define JDEROBOT_ReplayControllerInterfaceNTERFACE_H

#include <replayControl.h>
#include "SyncController.h"

namespace replayer{

    class ReplayControllerInterface: virtual public jderobot::replayControl{
    public:
        ReplayControllerInterface(replayer::SyncControllerPtr syncControllerPtr):
                controller(syncControllerPtr)
        {

        }
        ~ReplayControllerInterface(){

        }

        virtual bool pause(const Ice::Current&){
            controller->stop();
            return true;
        }
        virtual bool resume(const Ice::Current&){
            controller->resume();
            return true;
        }
        virtual void setReplay( bool replay, const Ice::Current&){
            controller->setRepeat(replay);
        }
        virtual bool setStep( Ice::Int step, const Ice::Current&){
            controller->setStep(step);
            return true;
        }
        virtual Ice::Long getTime(const Ice::Current&){
            return controller->getRelativeTime();
        }
        virtual bool goTo(Ice::Long seek, const Ice::Current&){
            //set the current position to seek
            return 0;
        }
        virtual jderobot::ReplayerStatus  getStatus(const Ice::Current&){
            return controller->getstatus();
        }


    private:
        replayer::SyncControllerPtr controller;
    };

}

#endif //JDEROBOT_ReplayControllerInterfaceNTERFACE_H
