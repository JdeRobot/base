import Ice
import rospy
from .ice.pose3dIceClient import Pose3dIceClient
from .ros.listenerPose3d import ListenerPose3d

def __getPoseIceClient(ic, prefix):
	'''
    Returns a Pose3D Ice Client. This function should never be used. Use getPose3dClient instead of this

    @param ic: Ice Communicator
    @param prefix: prefix name of client in config file

    @type ic: Ice Communicator
    @type prefix: String

    @return Pose3D Ice Client

    '''
	print("Receiving Pose3D from ICE interfaces")
	client = Pose3dIceClient(ic, prefix)
	client.start()
	return client

def __getListenerPose(ic, prefix):
	'''
    Returns a Pose3D ROS Subscriber. This function should never be used. Use getPose3dClient instead of this

    @param ic: Ice Communicator
    @param prefix: prefix name of client in config file

    @type ic: Ice Communicator
    @type prefix: String

    @return Pose3D ROS Subscriber

    '''
	print("Receiving Pose3D from ROS messages")
	prop = prop = ic.getProperties()
	topic = prop.getPropertyWithDefault(prefix+".Topic","");
	client = ListenerPose3d(topic)
	return client

def __Posedisabled(ic, prefix):
	'''
    Prints a warning that the client is disabled. This function should never be used. Use getPose3dClient instead of this

    @param ic: Ice Communicator
    @param prefix: prefix name of client in config file

	@type ic: Ice Communicator
    @type prefix: String

    @return None

    '''
	print(prefix + " Disabled")
	return None

def getPose3dClient (ic, prefix, node=None):
	'''
    Returns a Pose3D Client.

    @param ic: Ice Communicator
    @param prefix: prefix name of client in config file
    @param node: ROS node

    @type ic: Ice Communicator
    @type prefix: String
    @type node: ROS node

    @return None if pose3d is disabled

    '''
	prop = prop = ic.getProperties()
	server = prop.getPropertyAsIntWithDefault(prefix+".Server",0)

	cons = [__Posedisabled, __getPoseIceClient, __getListenerPose]

	return cons[server](ic, prefix)