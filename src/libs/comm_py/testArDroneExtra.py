#!/usr/bin/env python3
import config
import comm
import sys
import time
import signal





if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")


    client = jdrc.getArDroneExtraClient("Test.Extra")
    for i in range (10):
        client.takeoff()
        time.sleep(4)
        client.land()

    jdrc.destroy()