import sys, traceback, Ice
import jderobot
import numpy as np
from random import uniform
from random import randrange

bufferpoints = []
bufferline = []
bufferpose3D = []
id_list = []
obj_list = ["https://raw.githubusercontent.com/JdeRobot/JdeRobot/master/assets/gazebo/models/f1/meshes/F1.dae","model/Car.dae","https://raw.githubusercontent.com/JdeRobot/WebSim/master/bones/tronco2.obj"]
refresh = True


class PointI(jderobot.Visualization):
    def __init__(self):
	self.cont = 0

    def getSegment(self, current=None):
        rgblinelist = jderobot.bufferSegments()
	rgblinelist.buffer = []
	rgblinelist.refresh = refresh
	for i in bufferline[:]:
            rgblinelist.buffer.append(i)
            index = bufferline.index(i)
            del bufferline[index]
        return rgblinelist

    def drawPoint(self,point, color, current=None):
        print point

    def getPoints(self, current=None):
        rgbpointlist = jderobot.bufferPoints()
	rgbpointlist.buffer = []
	rgbpointlist.refresh = refresh
        for i in bufferpoints[:]:
            rgbpointlist.buffer.append(i)
            index = bufferpoints.index(i)
            del bufferpoints[index]
        return rgbpointlist

    def getObj3D(self, id, current=None):
        if len(obj_list) > self.cont:
            obj3D = jderobot.object3d()
            model = obj_list[self.cont].split(":")
            if model[0] == "https":
                obj3D.obj = obj_list[self.cont]
                model = obj_list[self.cont].split("/")
                name,form = model[len(model)-1].split(".")
                print "Sending model by url: " + name + "." + form
            else:
                obj = open(obj_list[self.cont], "r").read()
                name,form = obj_list[self.cont].split(".")
                obj3D.obj = obj
                print "Sending model by file: " + name + "." + form
            id_list.append(id)
            obj3D.id = id
            obj3D.format = form
            self.cont = self.cont + 1
            return obj3D

    def getPoseObj3DData(self, current=None):
	   for i in id_list[:]:
		   pose3d = jderobot.Pose3DData()
		   pose3d.x = randrange(0,20)
		   pose3d.y = randrange(0,20)
		   pose3d.z = randrange(0,20)
		   pose3d.h = randrange(-1,1)
		   pose3d.q0 = uniform(-1,1)
		   pose3d.q1 = uniform(-1,1)
		   pose3d.q2 = uniform(-1,1)
		   pose3d.q3 = uniform(-1,1)
		   objpose3d = jderobot.PoseObj3D()
		   objpose3d.id = id_list [id_list.index(i)]
		   objpose3d.pos = pose3d
		   bufferpose3D.append(objpose3d)
           return bufferpose3D

    def clearAll(self, current=None):
        print "Clear All"

def getbufferSegment (seg,color,plane):
    rgbsegment = jderobot.RGBSegment()
    rgbsegment.seg = seg
    if not plane:
	rgbsegment.seg.fromPoint.z = rgbsegment.seg.fromPoint.z * uniform(1, 10)
    	rgbsegment.seg.toPoint.z = rgbsegment.seg.toPoint.z * uniform(1, 10)
	rgbsegment.seg.fromPoint.y = rgbsegment.seg.fromPoint.y * uniform(1, 10)
    	rgbsegment.seg.toPoint.y = rgbsegment.seg.toPoint.y * uniform(1, 10)
	rgbsegment.seg.fromPoint.x = rgbsegment.seg.fromPoint.x * uniform(1, 10)
    	rgbsegment.seg.toPoint.x = rgbsegment.seg.toPoint.x * uniform(1, 10)
    rgbsegment.c = color
    bufferline.append(rgbsegment)

def getbufferPoint(point, color):
    rgbpoint = jderobot.RGBPoint()
    rgbpoint.x = point.x
    rgbpoint.y = point.y
    rgbpoint.z = point.z
    rgbpoint.r = color.r
    rgbpoint.g = color.g
    rgbpoint.b = color.b
    bufferpoints.append(rgbpoint)

try:
    endpoint = "default -h localhost -p 9957:ws -h localhost -p 11000"
    print "Connect: " + endpoint
    id = Ice.InitializationData()
    ic = Ice.initialize(None, id)
    adapter = ic.createObjectAdapterWithEndpoints("3DVizA", endpoint)
    object = PointI()
    adapter.add(object, ic.stringToIdentity("3DViz"))
    adapter.activate()
    ic.waitForShutdown()
except KeyboardInterrupt:
	del(ic)
	sys.exit()
