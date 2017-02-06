import Ice
import rospy
from .ice.laserIceClient import LaserIceClient
from .ros.listenerLaser import ListenerLaser


def init (ic):
	node = None
	rosserver = False

	prop = ic.getProperties()
	nodeName = prop.getPropertyWithDefault("NodeName", "JdeRobot")
	
	l = prop.getPropertiesForPrefix("")
	keys = l.keys()
	for i in keys:
		if (i.endswith(".Server") and l[i] == "1"):
			rosserver = True

	if (rosserver):
		node = rospy.init_node(nodeName, anonymous=True)

	return ic,node

	



