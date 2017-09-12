//
// Created by okan on 11.09.2017.
//

#ifndef JDEROBOT_CMAKEVARS_H
#define JDEROBOT_CMAKEVARS_H

#include <string>

class CMakeVars {
public:
    static const std::string& getInstallPrefix() {
        static std::string installPrefix("/opt/jderobot");
        return installPrefix;
    }
};

#endif //JDEROBOT_CMAKEVARS_H
