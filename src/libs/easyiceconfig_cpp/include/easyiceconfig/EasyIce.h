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

#ifndef EASYICECONFIG_EASYICE_H
#define EASYICECONFIG_EASYICE_H

/**
 * @mainpage  libEasyIceConfig
 *            A library to tame Ice.Config
 *
 * @author    Victor Arribas Raigadas, .varribas.urjc@gmail.com
 * @date      November 2015
 * @version    0.9.0 (alpha)
 */

#include <Ice/Ice.h>
#include <easyiceconfig/initializer.hpp>

namespace EasyIce{

inline
Ice::CommunicatorPtr initialize(int argc, char* argv[])
    {return easyiceconfig::initializer::initialize(argc,argv);}

inline
Ice::CommunicatorPtr initialize(Ice::StringSeq args)
    {return easyiceconfig::initializer::initialize(args);}

inline
Ice::PropertiesPtr createProperties(int argc, char* argv[])
    {return easyiceconfig::initializer::createProperties(argc, argv);}

inline
Ice::PropertiesPtr createProperties(Ice::StringSeq args)
    {return easyiceconfig::initializer::createProperties(args);}

}//NS


#endif // EASYICECONFIG_EASYICE_H
