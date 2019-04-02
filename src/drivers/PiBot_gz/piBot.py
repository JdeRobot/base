# -*- coding: utf-8 -*-
from jderobot_interfaces import Kibotics
import numpy
import threading
import sys
import comm
import config
import cv2
import imutils

GREEN_MIN = numpy.array([20, 50, 100],numpy.uint8)#numpy.array([48, 138, 138],numpy.uint8)
GREEN_MAX = numpy.array([90, 235, 210],numpy.uint8)#numpy.array([67, 177, 192],numpy.uint8)

BLUE_MIN = numpy.array([0, 255, 85],numpy.uint8)#numpy.array([104, 200, 42],numpy.uint8)
BLUE_MAX = numpy.array([179, 255, 255],numpy.uint8)#numpy.array([179, 255, 255],numpy.uint8)

RED_MIN = numpy.array([163, 209, 30],numpy.uint8)#numpy.array([48, 138, 138],numpy.uint8)
RED_MAX = numpy.array([179, 255, 255],numpy.uint8)#numpy.array([67, 177, 192],numpy.uint8)

ORANGE_MIN = numpy.array([0, 123, 165],numpy.uint8)#numpy.array([48, 138, 138],numpy.uint8)
ORANGE_MAX = numpy.array([179, 255, 255],numpy.uint8)#numpy.array([67, 177, 192],numpy.uint8)


class PiBot(Kibotics):

    '''
    Controlador para el Robot PiBot de JdeRobot-Kids
    '''
    def __init__(self):
        cfg = config.load("Kibotics.yml")
        print("En constructor")
        Kibotics.__init__(self)
        #cfg = config.load(cfg)
        
        #starting comm
        jdrc= comm.init(cfg, 'Kibotics.Sim')
        self.camera = jdrc.getCameraClient("Kibotics.Sim.Camera")
        self.motors = jdrc.getMotorsClient("Kibotics.Sim.Motors")    
        self.irLeft = jdrc.getIRClient("Kibotics.Sim.IRLeft")    
        self.irRight = jdrc.getIRClient("Kibotics.Sim.IRRight") 
        self.us = jdrc.getSonarClient("Kibotics.Sim.Sonar")    
        

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
        self.motors.sendW(0)
        self.motors.sendV(vel)

    def retroceder(self, vel):
        '''
        Función que hace retroceder al robot en línea recta a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de retroceso del robot (máximo 255)
        '''
        self.motors.sendW(0)
        self.motors.sendV(-vel)

    def parar(self):
        '''
        Función que hace detenerse al robot.
        '''
        self.motors.sendV(0)
        self.motors.sendW(0)

    def girarIzquierda(self, vel):
        '''
        Función que hace rotar al robot sobre sí mismo hacia la izquierda a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de giro del robot (máximo 255)
        '''
        self.motors.sendV(0)
        self.motors.sendW(vel)
        

    def girarDerecha(self, vel):
        '''
        Función que hace rotar al robot sobre sí mismo hacia la derecha a una velocidad dada como parámetro.
        @type vel: entero
        @param vel: velocidad de giro del robot (máximo 255)
        '''
        self.motors.sendV(0)
        self.motors.sendW(-vel)

    def move(self, velV, velW):
        '''
        Función que hace avanzar y girar al robot al mismo tiempo, según las velocidades V,W dadas como parámetro.
        @type velV, velW: entero
        @param velV, velW: velocidades de avance de motores izquierdo y derecho
        '''
        self.motors.sendV(velV)
        self.motors.sendW(velW)

    def dameImagen(self):
        '''
        Función que muestra la imagen percibida por la camara
        '''
        img = self.camera.getImage().data
        img = imutils.resize(img, width=400)

        return img
        #cv2.imshow("img", img)
        #cv2.waitKey(0)

    def leerIRSigueLineas(self):
        '''
        Función que retorna las lecturas del sensor siguelineas de la siguiente forma:
            0: ambos sensores sobre la linea
            1: solo sensor izquierdo sobre la linea
            2: solo sensor derecho sobre la linea 
            3: ambos sensores fuera de la linea
        '''
        lft = self.irLeft.getIRData().received
        rgt = self.irRight.getIRData().received
        value = -1
        if lft == 1 and rgt == 1:
            value = 0
        elif lft == 1 and rgt == 0:
            value = 1
        elif lft == 0 and rgt == 1:
            value = 2
        elif lft == 0 and rgt == 0:
            value = 3

        return value

    def leerUltrasonido(self):
        '''
        Función que retorna las lecturas del sensor ultrasonidos
        '''
        value = self.us.getSonarData().range
        return value

    def dameObjeto(self, lower=ORANGE_MIN, upper=ORANGE_MAX, showImageFiltered=False):

        image = self.dameImagen()
         # convert to the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # construct a mask for the color specified
        # then perform a series of dilations and erosions
        # to remove any small blobs left in the mask
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and
        # initialize the current center
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        area = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            area = M["m00"]

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle border
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                # and the centroid
                cv2.circle(image, center, 5, (0, 255, 255), -1)
        
        if showImageFiltered:
            # Control waitKey from outside, only for local executions, not jupyter.
            cv2.imshow("image_filtered", image)

        return center, area

    def mostrarImagen(self):
        None

    def dameSonarVisual(self):
        None

    def quienSoy(self):
        print ("Yo soy un robot simulado PiBot")

    @property
    def tipo(self):
        return self._tipo

    @tipo.setter
    def tipo(self, valor):
        self._tipo = valor
