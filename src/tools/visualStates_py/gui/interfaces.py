'''
   Copyright (C) 1997-2017 JDERobot Developers Team

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Library General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, see <http://www.gnu.org/licenses/>.

   Authors : Okan Aşık (asik.okan@gmail.com)

  '''
# a class to discover JdeRobot and ROS interfaces
import os

class Interfaces:

    interfaces = None

    @staticmethod
    def getInterfaces():
        if Interfaces.interfaces is None:
            os.system('/usr/local/bin/getinterfaces.sh /usr/local/include/jderobot/slice > /tmp/allInterfaces.txt')
            fp = open('/tmp/allInterfaces.txt')
            Interfaces.interfaces = {}
            for line in fp:
                data = line.strip("\n").split(' ')
                Interfaces.interfaces[data[0]] = data[1]

        return Interfaces.interfaces