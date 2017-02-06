import Ice
import rospy
from .ice.laserIceClient import LaserIceClient
from .ros.listenerLaser import ListenerLaser

def __getLaserIceClient(ic, prefix):
	print("Receiving LaserData from ICE interfaces")
	client = LaserIceClient(ic, prefix)
	client.start()
	return client

def __getListenerLaser(ic, prefix):
	print("Receiving LaserData from ROS messages")
	prop = prop = ic.getProperties()
	topic = prop.getPropertyWithDefault(prefix+".Topic","");
	client = ListenerLaser(topic)
	return client

def __Laserdisabled(ic, prefix):
	print("Laser Disabled")
	return None

def getLaserClient (ic, prefix, node=None):
	prop = prop = ic.getProperties()
	server = prop.getPropertyAsIntWithDefault(prefix+".Server",0)

	cons = [__Laserdisabled, __getLaserIceClient, __getListenerLaser]

	return cons[server](ic, prefix)