#!/bin/python2

import sys, time
import PiBot 

if __name__ == "__main__":

	robot = PiBot.dameRobot()

	# robot.avanzar(1)
	# time.sleep(2)
	# robot.parar()
	# time.sleep(2)
	# robot.retroceder(1)
	# time.sleep(2)
	# robot.dameImagen()
	while True:	
		robot.leerIRSigueLineas()
