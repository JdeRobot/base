/*
 *  Copyright (C) 2015-2016 JDE Developers Team
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

#ifndef RESOURCELOCATOR_H
#define RESOURCELOCATOR_H


#include <string>
#include <resourcelocator/stdutils.hpp>


namespace resourcelocator {

/**
 * @brief Find filename into all defined search paths.
 * Order is:
 * 1. current dir
 * 2. specified environment variable
 * 3. specified paths
 *
 * @return empty if file was not found.
 */
std::string findFile(const std::string filename, const std::string environment="", const std::string hardcored="");


}//NS


#endif // RESOURCELOCATOR_H
