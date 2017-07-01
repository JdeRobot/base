//
// Created by frivas on 3/04/17.
//

#include <pools/PoolWriteRGBD.h>
#include <pools/PoolWriteImages.h>
#include "RecorderInterface.h"



namespace recorder {

    RecorderInterface::RecorderInterface(PoolsManagerPtr& manager):
            manager(manager)
    {
    }

    bool RecorderInterface::saveLog(const ::std::string &name, ::Ice::Int seconds, const ::Ice::Current &ic)
    {
        bool ret = true;

        auto poolImages=manager->getPoolsByType(IMAGES);
        auto poolRGBD=manager->getPoolsByType(RGBD);

        for (size_t i=0; i< poolImages.size(); i++)
        {


            poolWriteImagesPtr pool = boost::dynamic_pointer_cast<poolWriteImages> (poolImages[i]);
            bool log = pool->startCustomLog(name, seconds);
            ret = ret && log;
        }

        for (size_t i=0; i< poolRGBD.size(); i++)
        {
            PoolWriteRGBDPtr pool = boost::dynamic_pointer_cast<PoolWriteRGBD> (poolRGBD[i]);
            bool log = pool->startCustomLog(name, seconds);
            ret = ret && log;
        }

        return ret;
    }

    bool RecorderInterface::saveVideo(const ::std::string &path, const ::std::string &name, ::Ice::Int seconds,
                                      const ::Ice::Current &ic) {
        bool ret = true;
        auto poolImages=manager->getPoolsByType(IMAGES);
        for (size_t i = 0; i < poolImages.size(); i++) {
            RecorderPoolPtr test;
            poolWriteImagesPtr pool = boost::dynamic_pointer_cast<poolWriteImages> (poolImages[i]);

            bool log = pool->startCustomVideo(path, name, seconds);
            ret = ret && log;
        }

        return ret;
    }



}