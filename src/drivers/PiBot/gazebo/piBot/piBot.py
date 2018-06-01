# -*- coding: utf-8 -*-
from JdeRobotKids import JdeRobotKids
import Ice
import numpy
import threading
import sys
import comm
import config
# import progeo

class PiBot:

    '''
    Controlador para el Robot PiBot de JdeRobot-Kids
    '''
    def __init__(self):
	 
        print("En constructor")
        cfg = config.load("JdeRobotKids.yml")
        
        #starting comm
        jdrc= comm.init(cfg, 'piBot')
        # self.camera = jdrc.getCameraClient("piBot.Camera")
        self.motors = jdrc.getMotorsClient("piBot.Motors")    

    def moverServo(self, *args):
        '''
        Función que hace girar al servo motor a un angulo dado como parámetro.
        @type args: lista
        @param args: lista de argumentos:
        args[0]: puerto al que esta conectado el controlador del servo
        args[1]: banco al que esta conectado el servo en el controlador
        args[2]: angulo de giro del servo. 0-180 grados. ¡PROBAR GIRO ANTES DE MONTAR EL SERVO!
        '''
        None

    def avanzar(self, vel):
        '''
        Función que hace avanzar al robot en línea recta a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de avance del robot (máximo 255)
        '''
        print("En avanzar")
        self.motors.sendV(vel)

    def retroceder(self, vel):
        '''
        Función que hace retroceder al robot en línea recta a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de retroceso del robot (máximo 255)
        '''
        print("En retroceder")

    def parar(self):
        '''
        Función que hace detenerse al robot.
        '''
        print("En parar")
        img = self.camera.getImage().data
        cv2.imshow("img", img)

    def girarIzquierda(self, vel):
        '''
        Función que hace rotar al robot sobre sí mismo hacia la izquierda a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de giro del robot (máximo 255)
        '''
        print("En girar izquierda")

    def girarDerecha(self, vel):
        '''
        Función que hace rotar al robot sobre sí mismo hacia la derecha a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de giro del robot (máximo 255)
        '''
        print("En girar derecha")

    def move(self, velV, velW):
        '''
        Función que hace avanzar y girar al robot al mismo tiempo, según las velocidades V,W dadas como parámetro.
        @type velV, velW: entero
        @param velV, velW: velocidades de avance de motores izquierdo y derecho
        '''
        print("En move")

	def quienSoy(self):
		print ("Yo soy un robot simulado PiBot")

	@property
	def tipo(self):
		return self._tipo

	@tipo.setter
	def tipo(self, valor):
		self._tipo = valor
