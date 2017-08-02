#!/usr/bin/env python2
import jderobotconfig as config
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import LaserData




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg["test"])

    client = jdrc.getLaserClient(ic, "Laser")
    client2 = jdrc.getLaserClient(ic, "Laser")

    for i in range (10):
        #print("client1", end=":")
        laser = client.getLaserData()
        #print(laser)
        time.sleep(1)


    for i in range (10):
        laser = client2.getLaserData()
        print(laser)
        time.sleep(1)

    jdrc.destroy()




    




