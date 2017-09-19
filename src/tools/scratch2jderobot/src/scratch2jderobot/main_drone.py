#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = "Raul Perula-Martinez"
__copyright__ = "JdeRobot project"
__credits__ = ["Raul Perula-Martinez"]
__license__ = "GPL v3"
__version__ = "0.0.0"
__maintainer__ = "Raul Perula-Martinez"
__email__ = "raules@gmail.com"
__status__ = "Development"


import easyiceconfig as EasyIce
import jderobotComm as comm
import os
import scratch

from drone import Drone

# get current working directory
path = os.getcwd()
open_path = path[:path.rfind('src')] + 'cfg/'
filename = 'drone.cfg'


if __name__ == '__main__':
    # loading the ICE and ROS parameters
    ic = EasyIce.initialize(['main_drone.py', open_path + filename])
    ic, node = comm.init(ic)

    # creating the object
    robot = Drone(ic)

    # executing the scratch program
    scratch.execute(robot)

    # destroying the common communications
    comm.destroy(ic, node)
