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
 *      Victor Arribas Raigadas <.varribas.urjc@gmail.com>
 *
 * Thanks to:
 *      http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring
 *      http://www.cplusplus.com/forum/general/1796/
 */


#include <easyiceconfig/stdutils.hpp>
#include <iostream>


int main(){
    std::string PATH = getEnvironmentVariable("PATH");
    std::vector<std::string> paths = std::split(PATH, ":");
    std::copy(paths.begin(), paths.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

    std::cout << std::fileexists("/bin/bash");
    std::cout << std::fileexists("/dev/null");
    std::cout << std::fileexists("/tmp/foo");
    for (std::string path : paths){
        std::cout << std::fileexists(path);
    }
}
