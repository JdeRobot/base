__author__ = 'robotica'

import sys, traceback, Ice
import jderobot, time

status = 0
ic = None
try:
    ic = Ice.initialize(sys.argv)
    base = ic.stringToProxy("PHPose3D:default -p 9998")
    datos = jderobot.Pose3DPrx.checkedCast(base)
    print datos
    if not datos:
        raise RuntimeError("Invalid proxy")

    while True:
        time.sleep(1)
        data = datos.getPose3DData()
        print data
except:
    traceback.print_exc()
    status = 1

if ic:
    # Clean up
    try:
        ic.destroy()
    except:
        traceback.print_exc()
        status = 1

sys.exit(status)
