#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

def execute(robot):
    try:
        while True:
            robot.move("%s", %s)
            time.sleep(5)
            robot.stop()
            time.sleep(1)
            robot.move("%s", %s)
            time.sleep(5)
            robot.stop()
            time.sleep(1)
            robot.turn("%s", %s)
            time.sleep(2)
            robot.stop()
            time.sleep(1)
            robot.turn("%s", %s)
            time.sleep(2)
            robot.stop()
            time.sleep(1)
        
    except KeyboardInterrupt:
        raise
