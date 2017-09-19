#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time


def execute(robot):
    try:
        robot.take_off()
        time.sleep(3)

        robot.go_up_down("up")
        time.sleep(4)
        robot.stop()
        time.sleep(5)

        while True:
            center = robot.color_object_centroid()
            if center != None:
                print "MOUSE DETECTED: ", center
            time.sleep(2)

    except KeyboardInterrupt:
        robot.land()
        time.sleep(1)
