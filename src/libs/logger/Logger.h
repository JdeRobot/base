/*
 *  Copyright (C) 2014 Jderobot Developers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/.
 *
 *  Authors : Roberto Calvo Palomino <rocapal [at] gsyc [dot] urjc [es]>
 *
 */

#ifndef LOG_H
#define LOG_H

#include <pthread.h>
#include <fstream>
#include <vector>
#include <Ice/Properties.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>


namespace jderobot
{

	class Logger {

	public:
		static void initialize(const std::string& appName){

            for (google::LogSeverity s = google::WARNING; s < google::NUM_SEVERITIES; s++)
                google::SetLogDestination(s, "");
            google::SetLogDestination(google::INFO, "log.log");
            FLAGS_alsologtostderr = 1;
			fLI::FLAGS_max_log_size=10;
		}

        static void initialize(const std::string& appName,const Ice::PropertiesPtr& prop, const std::string& prefixComponent){

            initialize(appName);

            std::string logFile = prop->getProperty(prefixComponent + ".Log.File.Name");
            if (logFile.size()==0)
                LOG(WARNING) << "You didn't set log file!";
            else
                jderobot::Logger::setFileLog(logFile);

            std::string logLevel = prop->getProperty(prefixComponent + ".Log.Level");
            if (logLevel.size()==0)
                LOG(WARNING) << "You didn't set *.Log.Level key!";
            else
                jderobot::Logger::setLogLevel(boost::lexical_cast<int>(logLevel));

            LOG(INFO) << "Logger:: logLevel=" + logLevel + " LogFile=" + logFile;
        }

        static void setFileLog(const std::string& logFile){
            for (google::LogSeverity s = google::WARNING; s < google::NUM_SEVERITIES; s++)
                google::SetLogDestination(s, "");
            google::SetLogDestination(google::INFO, logFile.c_str());
        }

        static void setLogLevel(int level){
            fLI::FLAGS_minloglevel=level;

        }
	};
}

#endif
