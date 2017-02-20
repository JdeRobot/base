#!/usr/bin/env python3
import easyiceconfig as EasyIce
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import CMDVel




if __name__ == '__main__':

    ic = EasyIce.initialize(sys.argv)
    ic, node = comm.init(ic)

    vel = CMDVel()
    vel.vx = 1
    vel.az = 0.1

    client = comm.getMotorsClient(ic, "kobukiViewer.Motors")
    #client2 = comm.getLaserClient(ic, "kobukiViewer.Laser")
    for i in range (10):
        client.sendVelocities(vel)
        time.sleep(1)

    comm.destroy(ic, node)