//
// Created by frivas on 3/04/17.
//

#ifndef JDEROBOT_RECORDERINTERFACE_H
#define JDEROBOT_RECORDERINTERFACE_H


#include <recorder.h>
#include <pools/PoolWriteImages.h>

namespace recorder {
    class RecorderInterface : virtual public jderobot::recorder {
    public:
        RecorderInterface(std::vector<RecorderPoolPtr> &poolImages);

        virtual bool saveLog(const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic);

        virtual bool
        saveVideo(const ::std::string &path, const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic);

    private:
        std::vector<RecorderPoolPtr> poolImages;
    };

}

#endif //JDEROBOT_RECORDERINTERFACE_H
