#!/usr/bin/python3
#
#  Copyright (C) 1997-2020 JDE Developers Team
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Shreyas Gokhale <shreyas6gokhale@gmail.com>


import rospy
from .ros.publisherMotors import PublisherMotors
from .ros.listenerLaser import ListenerLaser
from .ros.listenerPose3d import ListenerPose3d
import threading


class Connector:
    """
    ROS Connector Class
    """

    def __init__(self, config, prefix):
        """
        Connector constructor

        @param config: configuration of your application
        @type config: dict

        """
        self.config = config
        self.__state = ""
        self.__lock = threading.Lock()
        self.node_name = self.config[prefix]['NodeName']
        self.ros_node = rospy.init_node(self.node_name, anonymous=True)

    def destroy(self):
        """
        Shuts down ROS node

        """
        rospy.signal_shutdown("Node Closed")

    def getNode(self):
        return self.ros_node

    def getConfig(self):
        return self.config

    def getState(self):
        self.__lock.acquire()
        s = self.__state
        self.__lock.release()
        return s

    def setState(self, state):
        self.__lock.acquire()
        self.__state = state
        self.__lock.release()

    def getPoseListnerObject(self):
        topic = self.config[self.node_name]["Pose3D"]["Topic"]
        return ListenerPose3d(topic)

    def getMotorPubObject(self):
        topic = self.config[self.node_name]["Motors"]["Topic"]
        maxV = self.config[self.node_name]["Motors"]["maxV"]
        maxW = self.config[self.node_name]["Motors"]["maxW"]
        return PublisherMotors(topic, maxV, maxW)

    def getLaserListnerObject(self):
        topic = self.config[self.node_name]["Laser"]["Topic"]
        return ListenerLaser(topic)



