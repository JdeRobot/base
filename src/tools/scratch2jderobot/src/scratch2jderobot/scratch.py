#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

def execute(robot):
    try:
        robot.take_off()
        time.sleep(1)
        robot.move("forward")
        time.sleep(3)
        robot.stop()
        time.sleep(1)
        robot.move("left")
        time.sleep(3)
        robot.stop()
        time.sleep(1)
        robot.move("back")
        time.sleep(3)
        robot.stop()
        time.sleep(1)
        robot.move("right")
        time.sleep(3)
        robot.stop()
        time.sleep(1)
        robot.land()
        time.sleep(1)
    except KeyboardInterrupt:
        raise
