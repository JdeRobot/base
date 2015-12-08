//
// Created by frivas on 8/12/15.
//

#include "ReplayControlerClientHDL.h"


namespace  jderobot {

    ReplayControlerClientHDL::ReplayControlerClientHDL(const Ice::CommunicatorPtr &ic, const std::string &strProxy, bool stringIsAlreadyProxy):CommonHandler<jderobot::replayControlPrx>(ic,strProxy,stringIsAlreadyProxy){
    }

    jderobot::replayControlPrx ReplayControlerClientHDL::getProxy(){
        return CommonHandler<jderobot::replayControlPrx>::getproxy();
    }


    std::string ReplayControlerClientHDL::getStatusString(){
        std::string s;
        jderobot::ReplayerStatus value = getproxy()->getStatus();
        switch (value){
            case jderobot::WAITING:
                s="Waiting";
                break;
            case jderobot::PLAYING:
                s="Playing";
                break;
            case jderobot::PAUSED:
                s="Paused";
                break;
            case jderobot::FINISHED:
                s="Finished";
                break;
        }
        return s;
    }

}