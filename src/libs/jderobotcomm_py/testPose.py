#!/usr/bin/env python3
import easyiceconfig as EasyIce
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import Pose3d




if __name__ == '__main__':

    ic = EasyIce.initialize(sys.argv)
    ic, node = comm.init(ic)

    client = comm.getPose3dClient(ic, "kobukiViewer.Pose3D")

    for i in range (10):
        #print("client1", end=":")
        laser = client.getPose3d()
        print(laser)
        time.sleep(1)

    comm.destroy(ic, node)