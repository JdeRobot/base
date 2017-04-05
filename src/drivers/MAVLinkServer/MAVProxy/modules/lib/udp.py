#!/usr/bin/env python

"""
  MAVProxy UDP server
"""
import sys, socket

class UdpServer():
    '''
    a udp server for MAVProxy
    '''
    def __init__(self):
        self.client = None
        self.socket = None
        pass

    def connect(self, address, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((address, port))
        self.address = address
        self.port = port

    def readln(self):
        data, addr = self.socket.recvfrom(1024)
        self.client = addr
        return data

    def writeln(self, data):
        return self.socket.sendto(data, self.client)

    def bound(self):
        return self.socket is not None

    def connected(self):
        return self.client is not None


if __name__ == "__main__":
    # Test for UDP server
    pass
