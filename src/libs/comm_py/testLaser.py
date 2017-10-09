#!/usr/bin/env python2
import config
import comm
import sys
import time
import signal

from jderobotTypes import LaserData




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")

    client = jdrc.getLaserClient("Test.Laser")
    client2 = jdrc.getLaserClient("Test.Laser")

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




    




