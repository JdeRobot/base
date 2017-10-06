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


import config
import comm
import os
import scratch

from robot import Robot

# get current working directory
path = os.getcwd()
open_path = path[:path.rfind('src')] + 'cfg/'
filename = 'robot.yml'


if __name__ == '__main__':
    # loading the ICE and ROS parameters

    cfg = config.load(open_path + filename)

    #starting comm
    jdrc= comm.init(cfg, 'robot')

    # creating the object
    robot = Robot(jdrc)

    # executing the scratch program
    scratch.execute(robot)

    # destroying the common communications
    comm.destroy(ic, node)
