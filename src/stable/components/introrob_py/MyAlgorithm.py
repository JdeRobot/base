from sensors import sensor
import math
import jderobot
import cv2
import numpy as np


class MyAlgorithm():
    def __init__(self, sensor):
        self.sensor = sensor
        self.one=True

    def execute(self):
        # Add your code here
        tmp = self.sensor.getNavdata()
        if tmp!= None:
            print "State: " +str(tmp.state)
            print "Altitude: " +str(tmp.altd)
            print "Vehicle: " +str(tmp.vehicle)
            print "Battery %: " +str(tmp.batteryPercent)

