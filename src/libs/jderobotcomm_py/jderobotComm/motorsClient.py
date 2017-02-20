import Ice
import rospy
from .ice.motorsIceClient import MotorsIceClient
from .ros.publisherMotors import PublisherMotors

def __getMotorsIceClient(ic, prefix):
	print("Publishing Motors with ICE interfaces")
	client = MotorsIceClient(ic, prefix)
	client.start()
	return client

def __getPublisherMotors(ic, prefix):
	print("Publishing Motors with ROS messages")
	prop = prop = ic.getProperties()
	topic = prop.getPropertyWithDefault(prefix+".Topic","");
	client = PublisherMotors(topic)
	return client

def __Motorsdisabled(ic, prefix):
	print(prefix + " Disabled")
	return None

def getMotorsClient (ic, prefix, node=None):
	prop = prop = ic.getProperties()
	server = prop.getPropertyAsIntWithDefault(prefix+".Server",0)

	cons = [__Motorsdisabled, __getMotorsIceClient, __getPublisherMotors]

	return cons[server](ic, prefix)