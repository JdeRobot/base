#!/usr/bin/env python3
import easyiceconfig as EasyIce
import jderobotComm as comm
import sys


if __name__ == '__main__':
    ic = EasyIce.initialize(sys.argv)
    comm.init(ic)

