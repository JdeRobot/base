#!/usr/bin/env python3
import config
import comm
import sys
import time
import signal

from jderobotTypes import CMDVel




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")

    vel = CMDVel()
    vel.vx = 1
    vel.az = 0.1

    client = jdrc.getCMDVelClient("Test.CMDVel")
    for i in range (10):
        client.setCMD(vel)
        client.sendVelocities()
        time.sleep(1)

    jdrc.destroy()