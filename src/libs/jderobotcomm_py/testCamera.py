#!/usr/bin/env python2
import jderobotconfig as config
import jderobotComm as comm
import sys
import time
import signal

from jderobotTypes import Image




if __name__ == '__main__':

    cfg = config.load(sys.argv[1])
    jdrc= comm.init(cfg["test"])

    client = jdrc.getCameraClient("Camera1")

    for i in range (10):
        #print("client1", end=":")
        image = client.getImage()
        print(image)
        time.sleep(1)

    client.stop()


    jdrc.destroy()




    




