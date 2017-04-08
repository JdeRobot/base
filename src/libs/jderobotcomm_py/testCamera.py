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

    client = comm.getCameraClient(ic, "kobukiViewer.Camera1")

    for i in range (10):
        #print("client1", end=":")
        image = client.getImage()
        print(image)
        time.sleep(1)

    client.stop()


    comm.destroy(ic, node)




    




