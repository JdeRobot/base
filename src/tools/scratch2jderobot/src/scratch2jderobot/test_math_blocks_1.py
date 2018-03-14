#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import config
import sys
import comm
import os
import yaml

import math

from drone import Drone
from robot import Robot

def execute(robot):
    try:
        if 2 == 2 and 3 > 2:
            x = math.sqrt(9)
            x = math.sin(9)
            x = math.cos(9)
            x = math.tan(9)
            x = math.asin(9)
            x = math.acos(9)
            x = math.atan(9)
            x = math.log(9)
            x = math.log10(9)
            x = abs(9)
            x = math.floor(9)
            x = math.ceil(9)
        
    except KeyboardInterrupt:
        raise

if __name__ == '__main__':
    if len(sys.argv) == 2:
        path = os.getcwd()
        open_path = path[:path.rfind('src')] + 'cfg/'
        filename = sys.argv[1]

    else:
        sys.exit("ERROR: Example:python my_generated_script.py cfgfile.yml")

    # loading the ICE and ROS parameters
    cfg = config.load(open_path + filename)
    stream = open(open_path + filename, "r")
    yml_file = yaml.load(stream)

    for section in yml_file:
        if section == 'drone':
            #starting comm
            jdrc = comm.init(cfg,'drone')

            # creating the object
            robot = Drone(jdrc)

            break
        elif section == 'robot':
            #starting comm
            jdrc = comm.init(cfg,'robot')

            # creating the object
            robot = Robot(jdrc)

            break
    # executing the scratch program
    execute(robot)

