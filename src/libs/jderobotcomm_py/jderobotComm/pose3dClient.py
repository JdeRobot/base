import Ice
import rospy
from .ice.pose3dIceClient import Pose3dIceClient
from .ros.listenerPose3d import ListenerPose3d

def __getPoseIceClient(ic, prefix):
	print("Receiving Pose3D from ICE interfaces")
	client = Pose3dIceClient(ic, prefix)
	client.start()
	return client

def __getListenerPose(ic, prefix):
	print("Receiving Pose3D from ROS messages")
	prop = prop = ic.getProperties()
	topic = prop.getPropertyWithDefault(prefix+".Topic","");
	client = ListenerPose3d(topic)
	return client

def __Posedisabled(ic, prefix):
	print(prefix + " Disabled")
	return None

def getPose3dClient (ic, prefix, node=None):
	prop = prop = ic.getProperties()
	server = prop.getPropertyAsIntWithDefault(prefix+".Server",0)

	cons = [__Posedisabled, __getPoseIceClient, __getListenerPose]

	return cons[server](ic, prefix)