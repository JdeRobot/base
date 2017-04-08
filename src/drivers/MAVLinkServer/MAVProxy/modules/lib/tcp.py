#!/usr/bin/env python

"""
  MAVProxy TCP server
"""
import sys, socket

class TcpServer():
    '''
    a tcp server for MAVProxy
    '''
    def __init__(self):
        self.client = None
        self.socket = None
        self.conn = None
        pass

    def connect(self, address, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((address, port))
        self.address = address
        self.port = port
        self.socket.listen(1)
        self.conn, self.client = self.socket.accept()

    def readln(self):
        data = self.conn.recv(1024)
        return data

    def writeln(self, data):
        return self.conn.send(data)

    def connected(self):
        return self.conn is not None


if __name__ == "__main__":
    # Test for TCP server
    pass
