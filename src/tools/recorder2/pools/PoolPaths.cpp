//
// Created by frivas on 8/04/17.
//

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include "PoolPaths.h"


namespace recorder {
    PoolPaths::PoolPaths(const std::string &baseLogPath):
            baseLogPath(baseLogPath)
    {
        size_t pos = this->baseLogPath.find_last_of("/");
        if(pos == this->baseLogPath.length()-1)
        {
            this->baseLogPath.erase(this->baseLogPath.length()-1);
        }


        this->typesSufix[LASERS] = "lasers";
        this->typesSufix[POSE3DENCODERS] = "pose3dencoders";
        this->typesSufix[POSE3D] = "pose3d";
        this->typesSufix[ENCODERS] = "encoders";
        this->typesSufix[POINTCLOUD] = "pointClouds";
        this->typesSufix[IMAGES] = "images";

        this->typesSufixIndependent[LASERS] = "laser";
        this->typesSufixIndependent[POSE3DENCODERS] = "pose3dencoder";
        this->typesSufixIndependent[POSE3D] = "pose3d";
        this->typesSufixIndependent[ENCODERS] = "encoder";
        this->typesSufixIndependent[POINTCLOUD] = "pointCloud";
        this->typesSufixIndependent[IMAGES] = "camera";
        testPathAndCreateIfNotExists(this->baseLogPath);
    }

    std::string PoolPaths::getPathSeparator() {
        return "/";
    }

    void PoolPaths::testPathAndCreateIfNotExists(const std::string &path) {
        boost::filesystem::path p(path);
        if (!boost::filesystem::exists(p)) {
            boost::filesystem::create_directories(p);
        }
    }

    void PoolPaths::createRootPath(RECORDER_POOL_TYPE type) {
        std::string separator=getPathSeparator();
        std::string typeMainTypePath = this->baseLogPath + separator + this->typesSufix[type];
        testPathAndCreateIfNotExists(typeMainTypePath);
    }

    void PoolPaths::createDevicePath(RECORDER_POOL_TYPE type, int id) {
        createRootPath(type);
        std::string separator = getPathSeparator();
        std::string typeMainTypePath = this->baseLogPath + separator + this->typesSufix[type];
        std::stringstream ss;
        ss << id;
        testPathAndCreateIfNotExists(typeMainTypePath + separator + this->typesSufixIndependent[type] + ss.str());
    }

    std::string PoolPaths::getDeviceLogFilePath(RECORDER_POOL_TYPE type, int id) {
        return getDeviceLogPath(type,id) + this->typesSufixIndependent[type] + "Data.jde";
    }

    std::string PoolPaths::getDeviceLogPath(RECORDER_POOL_TYPE type, int id) {
        std::string separator = getPathSeparator();
        std::stringstream ss;
        ss << this->baseLogPath << separator << this->typesSufix[type] << separator << this->typesSufixIndependent[type] << id << separator;
        return ss.str();
    }

    std::string PoolPaths::getCustomLogFilePath(RECORDER_POOL_TYPE type, int id, const std::string &logname) {
        std::string logPath= getCustomLogPath(type,id,logname);

        return logPath + this->typesSufixIndependent[type] + "Data.jde";

    }

    std::string PoolPaths::getCustomLogPath(RECORDER_POOL_TYPE type, int id, const std::string &logname) {
        std::string separator = getPathSeparator();
        std::string rootCustomLogPath= this->baseLogPath + "-" + logname;




        this->testPathAndCreateIfNotExists(rootCustomLogPath);
        rootCustomLogPath+= separator + this->typesSufix[type] + separator;
        this->testPathAndCreateIfNotExists(rootCustomLogPath);
        std::stringstream ss;
        ss << rootCustomLogPath <<  this->typesSufixIndependent[type] << id << separator;
        this->testPathAndCreateIfNotExists(ss.str());
        return ss.str();
    }
}