/*
 *  Copyright (C) 1997-2017 JDE Developers Team
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
 *  Authors :
 *       Aitor Martinez Fernandez <aitor.martinez.fernandez@gmail.com>
 */

#include "jderobot/config/class.hpp"

namespace JdeRobotConfig{


Config::Config(){
}

Config::Config(YAML::Node node){
    this->node = node;
}

std::string 
Config::asString(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<std::string>();
}

float 
Config::asFloat(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<float>();
}

int 
Config::asInt(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<int>();
}

double 
Config::asDouble(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<double>();
}

YAML::Node
Config::getNode(){
    
    return node;
}



YAML::Node 
Config::searchNode(YAML::Node n, std::vector<std::string> names){
    YAML::Node nod = n[names[0]];
    names.erase(names.begin()); 

    if (names.size()>0){
        return this->searchNode(nod, names);
    }else{
        return nod;
    }
}


}//NS