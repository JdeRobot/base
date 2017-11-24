#!/usr/bin/env python2
import config
import comm
import sys
import time
import signal

from jderobotTypes import NavdataData




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg, "Test")

    client = jdrc.getNavdataClient("Test.Navdata")

    data = client.getNavData()
    print (data)

    jdrc.destroy()