/*
 *  Copyright (C) 1997-2016 JDE Developers Team
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

#ifndef JDEROBOTTYPES_POSE3D_H
#define JDEROBOTTYPES_POSE3D_H

#include <vector>

namespace JdeRobotTypes {

	class Pose3d {
	public:
		float x = 0; /**< %X coord [meters] */
	    float y = 0; /**< %Y coord [meters] */
	    float z = 0; /**< %Z coord [meters] */
	    float h = 0; /**< %H param */
	    float yaw = 0; /**< %Yaw angle[rads] */
	    float pitch = 0; /**< %Pitch angle[rads] */
	    float roll = 0; /**< %Roll angle[rads] */
	    std::vector<float> q = {0,0,0,0}; /**< %Quaternion */
	    double timeStamp = 0; /**< %Time stamp [s] */
	};


} //NS

#endif // JDEROBOTTYPES_POSE3D_H