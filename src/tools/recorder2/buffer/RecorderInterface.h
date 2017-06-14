//
// Created by frivas on 3/04/17.
//

#ifndef JDEROBOT_RECORDERINTERFACE_H
#define JDEROBOT_RECORDERINTERFACE_H


#include <recorder.h>
#include <pools/PoolsManager.h>

namespace recorder {
    class RecorderInterface : virtual public jderobot::recorder {
    public:
        RecorderInterface(PoolsManagerPtr& manager);

        virtual bool saveLog(const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic);

        virtual bool
        saveVideo(const ::std::string &path, const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic);

    private:
        PoolsManagerPtr manager;


    };

}

#endif //JDEROBOT_RECORDERINTERFACE_H
