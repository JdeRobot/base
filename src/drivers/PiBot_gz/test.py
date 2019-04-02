#!/bin/python2

import sys, time
from piBot import PiBot

if __name__ == "__main__":

	robot = PiBot()

	# robot.avanzar(1)
	# time.sleep(2)
	# robot.parar()
	# time.sleep(2)
	# robot.retroceder(1)
	# time.sleep(2)
	# robot.dameImagen()
	while True:	
		robot.leerIRSigueLineas()
