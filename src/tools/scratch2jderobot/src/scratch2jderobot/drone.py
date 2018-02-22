#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import imutils
import cv2


from parallelIce.cameraClient import CameraClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.navDataClient import NavDataClient
from parallelIce.pose3dClient import Pose3DClient

# define the lower and upper boundaries of the basic colors
GREEN_RANGE = ((29, 86, 6), (64, 255, 255))
RED_RANGE = ((139, 0, 0), (255, 160, 122))
BLUE_RANGE = ((0, 128, 128), (65, 105, 225))

import math
import time
import imutils
import cv2
import comm
import numpy as np

from jderobotTypes import CMDVel
from jderobotTypes import Pose3d
from jderobotTypes import Image

# define the lower and upper boundaries of the basic colors
GREEN_RANGE = ((29, 86, 6), (64, 255, 255))
RED_RANGE = ((139, 0, 0), (255, 160, 122))
BLUE_RANGE = ((0, 128, 128), (65, 105, 225))


class Drone():

    """
    Drone class.
    """

    def __init__(self, jdrc):
        """
        Init method.
        @param jdrc:
        """

    	#variables
        self.frontalCamera = False

    	#get clients
        self.__pose3d_client = jdrc.getPose3dClient("drone.Pose3D")
        self.__camera_client = jdrc.getCameraClient("drone.Camera1")
        self.__cmdvel_client = jdrc.getCMDVelClient("drone.CMDVel")
        self.__extra_client = jdrc.getArDroneExtraClient("drone.Extra")
        self.__navdata_client = jdrc.getNavdataClient("drone.Navdata")

    def close(self):
        """
        Close communications with servers.
        """

        self.__camera_client.stop()
        self.__navdata_client.stop()
        self.__pose3d_client.stop()



    def get_pose3d(self):
        """
        Get the value of odometry sensor.

        @return: return the asked value.
        """

        return self.__pose3d_client.getPose3d()


    def toggleCam(self):
        self.frontalCamera = True
        self.__extra_client.toggleCam()


    def detect_object(self, color):
        """
        Detect an object using the camera.

        @param color: color to detect

        @return: size and center of the object detected in the frame
        """

        if not self.frontalCamera:
            self.toggleCam()
        # define the lower and upper boundaries of the basic colors
        GREEN_RANGE = ((29, 86, 6), (64, 255, 255))
        RED_RANGE = ((139, 0, 0), (255, 160, 122))
        BLUE_RANGE = ((0, 128, 128), (65, 105, 225))

        # initialize the values in case there is no object
        x_position = 0
        y_position = 0
        size = 0

        # chose the color to find
        if color == "red":
            color_range = RED_RANGE
        if color == "green":
            color_range = GREEN_RANGE
        if color == "blue":
            color_range = BLUE_RANGE

        # get image type from camera
        image = self.__camera_client.getImage()

        # apply color filters to the image
        filtered_image = cv2.inRange(image.data, color_range[0], color_range[1])
        rgb = cv2.cvtColor(image.data, cv2.COLOR_BGR2RGB)


        # Apply threshold to the masked image
        ret,thresh = cv2.threshold(filtered_image,127,255,0)
        im,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        # Find the index of the largest contour
        for c in contours:
            if c.any != 0:
                areas = [cv2.contourArea(c) for c in contours]
                max_index = np.argmax(areas)
                cnt=contours[max_index]
                if max(areas) > 0.0:
                    x,y,w,h = cv2.boundingRect(cnt)
                    cv2.rectangle(rgb,(x,y),(x+w,y+h),(0,255,0),2)
                    x_position = (w/2)+x
                    y_position = (h/2)+y
                    size = w*h

        # show the frame to our screen
        cv2.imshow("Frame", rgb)
        key = cv2.waitKey(1) & 0xFF

        return size, x_position, y_position


    def go_up_down(self, direction):
        """
        Set the vertical movement of the drone.

        @param direction: direction of the move. Options: forward (default), back.
        """

        # set default velocity (m/s)
        vz = 2.0

        if direction == "down":
            vz = -vz

        # assign velocity
        self.__cmdvel_client.setVZ(vz)

        # publish movement
        self.__cmdvel_client.sendVelocities()

    def move_vector(self, velocities):
        """
        Set the movements of the drone.

        @param velocities: a scratch list [x,z,yaw] with the velocities in m/s.
        """
        vx = float(velocities[0])
        vz = float(velocities[1])
        vyaw = float(velocities[2])
        print "velocities vector:","x:",vx,"z:",vz,"yaw:",vyaw
        self.__cmdvel_client.setVX(vx)
        self.__cmdvel_client.setVZ(vz)
        self.__cmdvel_client.setYaw(vyaw)

        # publish movement
        self.__cmdvel_client.sendVelocities()
    #
    # def move(self, direction, vel=None):
    #     """
    #     Set the horizontal movement of the drone.
    #
    #     @param direction: direction of the move. Options: forward (default), back.
    #     @param vel: a number with the velocity in m/s. Default: 1 m/s.
    #     """
    #
    #     if vel == None:
    #         vel = 1
    #     # set different direction
    #     if direction == "back":
    #         self.__cmdvel_client.setVX(-vel)
    #     elif direction == "forward":
    #         self.__cmdvel_client.setVX(vel)
    #     elif direction == "left":
    #         self.__cmdvel_client.setVY(vel)
    #     elif direction == "right":
    #         self.__cmdvel_client.setVY(-vel)
    #     elif direction == "down":
    #         self.__cmdvel_client.setVZ(-vel)
    #     elif direction == "up":
    #         self.__cmdvel_client.setVZ(vel)
    #     print direction
    #
    #     # publish movement
    #     self.__cmdvel_client.sendVelocities()
    #
    # def turn(self, direction, vel=None):
    #     """
    #     Set the angular velocity.
    #
    #     @param direction: direction of the move. Options: left (default), right.
    #     @param vel: a number with the velocity in m/s. Default: 1 m/s.
    #     """
    #     if vel == None:
    #         vel = 0.5
    #     # set default velocity (m/s)
    #     yaw = vel * math.pi
    #
    #     if direction == "right":
    #         yaw = -yaw
    #
    #     # assign velocity
    #     self.__cmdvel_client.setYaw(yaw)
    #
    #     # publish movement
    #     self.__cmdvel_client.sendVelocities()

    def stop(self):
        """
        Set all velocities to zero.
        """

        self.__cmdvel_client.setVX(0)
        self.__cmdvel_client.setVY(0)
        self.__cmdvel_client.setVZ(0)
        self.__cmdvel_client.setYaw(0)

        self.__cmdvel_client.sendVelocities()

    def take_off(self):
        """
        Send the take off command.
        """

        self.__extra_client.takeoff()
        time.sleep(1)

    def land(self):
        """
        Send the land command.
        """

        self.__extra_client.land()
        time.sleep(1)
