//
// Created by frivas on 3/04/17.
//

#include "RecorderInterface.h"



namespace recorder {

    RecorderInterface::RecorderInterface(std::vector<poolWriteImages *> &poolImages):
            poolImages(poolImages)
    {

    }

    bool RecorderInterface::saveLog(const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic)
    {
        bool ret = true;
        for (int i=0; i< poolImages.size(); i++)
        {
            bool log = poolImages[i]->startCustomLog(name, seconds);
            ret = ret && log;
        }

        return ret;
    }

    bool RecorderInterface::saveVideo(const ::std::string &path, const ::std::string &name, ::Ice::Int seconds,
                                      const ::Ice::Current &ic) {
        bool ret = true;
        for (int i = 0; i < poolImages.size(); i++) {
            bool log = poolImages[i]->startCustomVideo(path, name, seconds);
            ret = ret && log;
        }

        return ret;
    }



}