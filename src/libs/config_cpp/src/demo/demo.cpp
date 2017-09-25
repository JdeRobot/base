#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <jderobot/config/config.h>


int 
main( int argc, const char* argv[] ){
    std::string filename (argv[1]);
    JdeRobotConfig::Config props = JdeRobotConfig::load(filename);
    std::cout << props << std::endl;
    std::cout << props.asString("Demo.Motors.Proxy")<< std::endl;
    std::cout << props.asFloat("Demo.Motors.maxW")<< std::endl;
    std::cout << props.asInt("Demo.Motors.maxV")<< std::endl;


    return 0;
}
