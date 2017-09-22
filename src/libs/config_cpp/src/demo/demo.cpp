#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <jderobot/config/config.h>


int 
main( int argc, const char* argv[] ){
    std::string filename (argv[1]);
    YAML::Node props = JdeRobotConfig::load(filename);
    std::cout << props << std::endl;

    std::cout << props["Demo.Name"]<< std::endl;
    //std::cout << props.IsScalar()<< std::endl;
    //std::cout << props.as<int>()<< std::endl;

    return 0;
}
