//
// Created by frivas on 8/04/17.
//

#ifndef JDEROBOT_POOLPATHS_H
#define JDEROBOT_POOLPATHS_H


#include "PoolsManager.h"

namespace recorder{
    class PoolPaths {
    public:
        PoolPaths(const std::string& baseLogPath);
        void createRootPath(RECORDER_POOL_TYPE type);
        void createDevicePath(RECORDER_POOL_TYPE type, int id);
        std::string getDeviceLogFilePath(RECORDER_POOL_TYPE type, int id);
        std::string getDeviceLogPath(RECORDER_POOL_TYPE type, int id);
        std::string getCustomLogFilePath(RECORDER_POOL_TYPE type, int id, const std::string& logname);
        std::string getCustomLogPath(RECORDER_POOL_TYPE type, int id, const std::string& logname);


    private:
        std::map<RECORDER_POOL_TYPE, std::string> typesSufix;
        std::map<RECORDER_POOL_TYPE, std::string> typesSufixIndependent;
        std::string baseLogPath;
        std::string getPathSeparator();
        void testPathAndCreateIfNotExists(const std::string& path);
    };

}


#endif //JDEROBOT_POOLPATHS_H
