import Ice
import rospy
from .ice.cameraIceClient import CameraIceClient
from .ros.listenerCamera import ListenerCamera

def __getCameraIceClient(ic, prefix):
	print("Receiving " + prefix + " Image from ICE interfaces")
	client = CameraIceClient(ic, prefix)
	client.start()
	return client

def __getListenerCamera(ic, prefix):
	print("Receiving " + prefix + " Image from ROS messages")
	prop = prop = ic.getProperties()
	topic = prop.getPropertyWithDefault(prefix+".Topic","");
	client = ListenerCamera(topic)
	return client

def __Cameradisabled(ic, prefix):
	print("Camera Disabled")
	return None

def getCameraClient (ic, prefix, node=None):
	prop = prop = ic.getProperties()
	server = prop.getPropertyAsIntWithDefault(prefix+".Server",0)

	cons = [__Cameradisabled, __getCameraIceClient, __getListenerCamera]

	return cons[server](ic, prefix)