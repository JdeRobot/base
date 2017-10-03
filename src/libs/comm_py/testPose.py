#!/usr/bin/env python3
import config
import comm
import sys
import time
import signal

from jderobotTypes import Pose3d




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")

    client = jdrc.getPose3dClient("Test.Pose3D")

    for i in range (10):
        #print("client1", end=":")
        laser = client.getPose3d()
        print(laser)
        time.sleep(1)

    jdrc.destroy()