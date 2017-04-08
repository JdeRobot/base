#!/usr/bin/env python3
import easyiceconfig as EasyIce
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import LaserData




if __name__ == '__main__':

    ic = EasyIce.initialize(sys.argv)
    ic, node = comm.init(ic)

    client = comm.getLaserClient(ic, "kobukiViewer.Laser")
    client2 = comm.getLaserClient(ic, "kobukiViewer.Laser")

    for i in range (10):
        #print("client1", end=":")
        laser = client.getLaserData()
        #print(laser)
        time.sleep(1)


    for i in range (10):
        laser = client2.getLaserData()
        print(laser)
        time.sleep(1)

    comm.destroy(ic, node)




    




