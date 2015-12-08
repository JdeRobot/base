//
// Created by frivas on 8/12/15.
//

#ifndef JDEROBOT_REPLAYCONTROLERCLIENTHDL_H
#define JDEROBOT_REPLAYCONTROLERCLIENTHDL_H

#include <jderobot/replayControl.h>
#include <boost/shared_ptr.hpp>
#include <jderobot/ns/ns.h>
#include <jderobotHandlers/common/CommonHandler.h>


namespace  jderobot {

    class ReplayControlerClientHDL;
    typedef boost::shared_ptr<ReplayControlerClientHDL> ReplayControlerClientHDLPtr;

    class ReplayControlerClientHDL: public CommonHandler<jderobot::replayControlPrx>{
    public:
        // look in properties means that srtProxy is a key inside the properties file not the url of the proxy
        ReplayControlerClientHDL(const Ice::CommunicatorPtr &ic, const std::string &strProxy, bool stringIsAlreadyProxy);
        jderobot::replayControlPrx getProxy();

        std::string getStatusString();
    };
}

#endif //JDEROBOT_REPLAYCONTROLERCLIENTHDL_H
