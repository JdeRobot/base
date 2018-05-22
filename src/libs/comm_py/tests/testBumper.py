#!/usr/bin/env python2
import config
import comm
import sys
import time
import signal

from jderobotTypes import BumperData




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")

    client = jdrc.getBumperClient("Test.Bumper")

    while True:
        #print("client1", end=":")
        bumper = client.getBumperData()
        print bumper
        if bumper.state == 1:
            print bumper
            break
        time.sleep(1)

    jdrc.destroy()