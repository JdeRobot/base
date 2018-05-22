/*
 *
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
 *
 *  Author: Francisco Rivas <franciscomiguel [dot] rivas [at] urjc [dot] es>
 *
 */

#ifndef RGBD_ICE
#define RGBD_ICE

#include <jderobot/image.ice>

module jderobot{
    struct rgbData {
        ImageData color;
        ImageData depth;
        Time timeStamp; 				/**< TimeStamp of Data */
    };


    interface rgbd{
        idempotent rgbData getData();

    };
};



#endif