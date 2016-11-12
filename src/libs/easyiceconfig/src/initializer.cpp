/*
 *  Copyright (C) 1997-2015 JDE Developers Team
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
 *       Victor Arribas Raigadas <.varribas.urjc@gmail.com>
 */


#include "easyiceconfig/initializer.hpp"


namespace easyiceconfig{
namespace initializer{

/**
 * Like Ice::initialize but with path locator.
 * Parses argv[0] like Ice::initialize (backward compatibility),
 * but it should only parse from argv[1]. Take this into account
 * for future.
 */
Ice::CommunicatorPtr
initialize(int argc, char* argv[]){
    Ice::StringSeq args(argv, argv+argc);
    return initialize(args);
}

Ice::CommunicatorPtr
initialize(Ice::StringSeq args){
    Ice::InitializationData id;
    id.properties = easyiceconfig::loader::initializeProperties(args);
    return Ice::initialize(id);
}

Ice::PropertiesPtr
createProperties(int argc, char* argv[]){
    Ice::StringSeq args(argv, argv+argc);
    return easyiceconfig::loader::initializeProperties(args);
}

Ice::PropertiesPtr
createProperties(Ice::StringSeq args){
    return easyiceconfig::loader::initializeProperties(args);
}

}}//NS
