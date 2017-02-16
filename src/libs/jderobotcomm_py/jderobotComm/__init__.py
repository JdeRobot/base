import Ice
import rospy

from .laserClient import getLaserClient
from .cameraClient import getCameraClient


def init (ic):
	node = None
	rosserver = False

	prop = ic.getProperties()
	nodeName = prop.getPropertyWithDefault("NodeName", "JdeRobot")
	
	l = prop.getPropertiesForPrefix("")
	keys = l.keys()
	for i in keys:
		if (i.endswith(".Server") and l[i] == "2"):
			rosserver = True

	if (rosserver):
		node = rospy.init_node(nodeName, anonymous=True)

	return ic,node

def destroy(ic=None, node=None):
	if node:
		rospy.signal_shutdown("Node Closed")
	if ic:
		ic.destroy()




	



