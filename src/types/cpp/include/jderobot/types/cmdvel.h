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

#ifndef JDEROBOTTYPES_CMDVEL_H
#define JDEROBOTTYPES_CMDVEL_H

namespace JdeRobotTypes {

	class CMDVel {
	public:
		float vx = 0; /**< % vel in x[m/s] (use this for V in wheeled robots)*/
	    float vy = 0; /**< %vel in y[m/s] */
	    float vz = 0; /**< %vel in z[m/s] */
	    float ax = 0; /**< %angular vel in X axis [rad/s] */
	    float ay = 0; /**< %angular vel in Y axis [rad/s] */
	    float az = 0; /**< %angular vel in Z axis [rad/s] (use this for W in wheeled robots)*/
	    double timeStamp = 0; /**< %Time stamp [s] */
	};


} //NS

#endif // JDEROBOTTYPES_CMDVEL_H