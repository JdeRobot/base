#!/usr/bin/env python3
import jderobotconfig as config
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import CMDVel




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg["test"])

    vel = CMDVel()
    vel.vx = 1
    vel.az = 0.1

    client = jdrc.getMotorsClient(ic, "Motors")
    for i in range (10):
        client.sendVelocities(vel)
        time.sleep(1)

    jdrc.destroy()