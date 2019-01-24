#(c) JdeRobot-kids 2018

# -*- coding: utf-8 -*-
import abc
from abc import ABCMeta


class Kibotics(object):
    __metaclass__ = ABCMeta

    @abc.abstractmethod
    def quienSoy(self):
        print ("Soy un tipo de robot abstracto")

    @abc.abstractmethod
    def avanzar(self, vel):
        print ("Avanzar abstracto")

    @abc.abstractmethod
    def retroceder(self, vel):
        print ("Retroceder abstracto")

    @abc.abstractmethod
    def parar(self):
        print ("Parar abstracto")

    @abc.abstractmethod
    def girarIzquierda(self, vel):
        print ("Girar izquierda abstracto")

    @abc.abstractmethod
    def girarDerecha(self, vel):
        print ("Girar derecha abstracto")

    @abc.abstractmethod
    def move(self, velV, velW):
        print ("Move abstracto")

    @abc.abstractmethod
    def leerUltrasonido(self):
        print ("leerUltrasonido abstracto")

    @abc.abstractmethod
    def leerIRSigueLineas(self):
        print ("leerIRSigueLineas abstracto")

    @abc.abstractmethod
    def moverServo(self, *args):
        print ("moverServo abstracto")

    def get_tipo(self):
        pass
    
    def set_tipo(self, valor):
        pass

    tipo = abc.abstractproperty(get_tipo, set_tipo)

    @abc.abstractmethod
    def dameImagen(self):
        print ("dameImagen abstracto")

    @abc.abstractmethod
    def mostrarImagen(self):
        print ("mostrarImagen abstracto")

    @abc.abstractmethod
    def dameSonarVisual (self, image, cameraModel):
        print ("dameSonarVisual abstracto")

    @abc.abstractmethod
    def dameObjeto(self, lower, upper, showImageFiltered):
        print ("dameObjeto abstracto")

