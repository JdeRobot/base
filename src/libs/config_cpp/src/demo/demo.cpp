#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <jderobot/config/config.h>


int 
main( int argc, char* argv[] ){
    Config::Properties props = Config::load(argc, argv);
    std::cout << props << std::endl;
    std::cout << props.asString("Demo.Motors.Proxy")<< std::endl;
    std::cout << props.asStringWithDefault("Demo.Motors.Proxy2", "Proxy2")<< std::endl;
    std::cout << props.asFloat("Demo.Motors.maxW")<< std::endl;
    std::cout << props.asInt("Demo.Motors.maxV")<< std::endl;


    return 0;
}
