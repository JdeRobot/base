#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <jderobot/config/config.h>


int 
main( int argc, const char* argv[] ){
    std::string filename (argv[1]);
    YAML::Node props = JdeRobotConfig::load(filename);
    std::cout << props << std::endl;

    return 0;
}
