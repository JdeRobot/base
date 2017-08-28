# a class to discover JdeRobot and ROS interfaces
import os

class Interfaces:

    interfaces = None

    @staticmethod
    def getInterfaces():
        if Interfaces.interfaces is None:
            os.system('/usr/local/bin/getinterfaces.sh /usr/local/include/jderobot/slice > /tmp/allInterfaces.txt')
            fp = open('/tmp/allInterfaces.txt')
            Interfaces.interfaces = {}
            for line in fp:
                data = line.strip("\n").split(' ')
                Interfaces.interfaces[data[0]] = data[1]

        return Interfaces.interfaces