# -*- coding: utf-8 -*-
from __future__ import print_function

from jderobot_interfaces import Kibotics
import numpy
import sys, time
import jderobot_config
#import progeo
import cv2
import time
import math
import yaml
#from odometry import Odometry
import RPi.GPIO as GPIO

from imutils.video import VideoStream
import imutils

'''
DEBUG
1: Inicialización driver
2: Inicialización sensores
10: Odometría
11: Finalización 
'''

DEBUG = [1,2,10,11]
#DEBUG = [1,2]	
#DEBUG = [10]	

ORANGE_MIN = numpy.array([0, 123, 165],numpy.uint8)#numpy.array([48, 138, 138],numpy.uint8)
ORANGE_MAX = numpy.array([179, 255, 255],numpy.uint8)#numpy.array([67, 177, 192],numpy.uint8)

def welcome():
        print("""
		.----------------.  .----------------.  .----------------.  .----------------.  .----------------.
		| .--------------. || .--------------. || .--------------. || .--------------. || .--------------. |
		| |   ______     | || |     _____    | || |   ______     | || |     ____     | || |  _________   | |
		| |  |_   __ \   | || |    |_   _|   | || |  |_   _ \    | || |   .'    `.   | || | |  _   _  |  | |
		| |    | |__) |  | || |      | |     | || |    | |_) |   | || |  /  .--.  \  | || | |_/ | | \_|  | |
		| |    |  ___/   | || |      | |     | || |    |  __'.   | || |  | |    | |  | || |     | |      | |
		| |   _| |_      | || |     _| |_    | || |   _| |__) |  | || |  \  `--'  /  | || |    _| |_     | |
		| |  |_____|     | || |    |_____|   | || |  |_______/   | || |   `.____.'   | || |   |_____|    | |
		| |              | || |              | || |              | || |              | || |              | |
		| '--------------' || '--------------' || '--------------' || '--------------' || '--------------' |
		'----------------'  '----------------'  '----------------'  '----------------'  '----------------' """)

class PiBot(Kibotics):

	'''
	Controlador para el Robot PiBot de JdeRobot-Kids
	Actualmente incluye dos funciones de procesamiento visual:
	- damePosicionDeObjetoDeColor
	- dameSonarVisual
	'''
	def __init__(self):
		cfg = config.load("Kibotics.yml")
		welcome()
		if 1 in DEBUG:
			print("[PiBot] Generando controlador en HW real...")

		try:
			Kibotics.__init__(self)

			self._GPIO = GPIO
			self._dit = pigpio.pi()
			self._tipo = "PiBot"
			self._frame = None

			cam = cfg.getProperty('Kibotics.Real.Camera')
			fps = cfg.getProperty('Kibotics.Real.FPS')
			w = cfg.getProperty('Kibotics.Real.Width')
			h = cfg.getProperty('Kibotics.Real.Height')
			d = cfg.getProperty('Kibotics.Real.Dist_ruedas')
			r = cfg.getProperty('Kibotics.Real.Radio_ruedas')

			if cam == "PiCam":
				if 2 in DEBUG:
					print("[PiBot] Iniciando Cámara: ", cam, "...")
					print("\t[Cámara] FPS: ", fps)
					print("\t[Cámara] Resolución: ("+str(w)+","+str(h)+")")

				self._videostream = VideoStream(usePiCamera=True, framerate=fps, resolution=(w,h)).start()
			
			time.sleep(2)
			self.odom = Odometry(d, r, self._GPIO)
			self._odometry_interval = 0.032


			if 1 in DEBUG:
				print("...controlador ACTIVO")

		except Exception as e:
			if 1 in DEBUG:
				print("...controlador ERROR!", str(e))


	def __del__(self):
		self._fin()


	def _fin(self):
		self.parar()
		self.odom.stopOdometry()	


	def _getOdometry(self):

		x,y,theta = self.odom.getOdometry()
		return (x,y,theta)


	def moverServo(self, *args):
		'''
		Función que hace girar al servo motor a un angulo dado como parámetro.
		@type args: lista
		@param args: lista de argumentos:
		args[0]: puerto al que esta conectado el controlador del servo
		args[1]: banco al que esta conectado el servo en el controlador
		args[2]: angulo de giro del servo. 0-180 grados. ¡PROBAR GIRO ANTES DE MONTAR EL SERVO!
		'''
		puerto = args[0]
		banco = args[1]
		angulo = args[2]/10

		self._GPIO.setmode(GPIO.BOARD)   #Ponemos la Raspberry en modo BOARD
		self._GPIO.setup(puerto,GPIO.OUT)	#Ponemos el pin args[0] como salida
		p = self._GPIO.PWM(puerto,50)		#Ponemos el pin args[0] en modo PWM y enviamos 50 pulsos por segundo
		p.start(7.5)			   #Enviamos un pulso del 7.5% para centrar el servo
		p.ChangeDutyCycle(angulo)
		time.sleep(0.5)


	def avanzar(self, vel):
		'''
		Función que hace avanzar al robot en línea recta a una velocidad dada como parámetro.
		@type vel: entero
		@param vel: velocidad de avance del robot (máximo 255)
		'''
		self.move(vel, 0)


	def retroceder(self, vel):
		'''
		Función que hace retroceder al robot en línea recta a una velocidad dada como parámetro.
		@type vel: entero
		@param vel: velocidad de retroceso del robot (máximo 255)
		'''
		self.move(-vel, 0)


	def parar(self):
		'''
		Función que hace detenerse al robot.
		'''
		# Puertos de datos para servos izquierdo y derecho
		puertoL = 4
		puertoR = 18
		self._dit.set_servo_pulsewidth(puertoL, 1520) #parado 1525
		self._dit.set_servo_pulsewidth(puertoR, 1520) #parado 1510


	def girarIzquierda(self, velW):
		'''
		Función que hace rotar al robot sobre sí mismo hacia la izquierda a una velocidad dada como parámetro.
		@type vel: entero
		@param vel: velocidad de giro del robot (máximo 255)
		'''
		self.move(0, velW)


	def girarDerecha(self, velW):
		'''
		Función que hace rotar al robot sobre sí mismo hacia la derecha a una velocidad dada como parámetro.
		@type vel: entero
		@param vel: velocidad de giro del robot (máximo 255)
		'''
		self.move(0, -velW)


	def _posicion_alcanzada(self, dif, p, e):
		return abs(dif) >= (abs(p) - e) and abs(dif) <= (abs(p) + e)


	def moverHasta(self, pos):
		'''
		Avanza o retrocede a una posicion dada
		'''
		Timeout = 10 #[s] Tiempo maximo que va a estar bloqueada la funcion
		ErrorRange = 0.02 #Margen de error para la posicion que ha de alcanzar
		Vel = 0.08 #Velocidad a la que se movera el robot

		finish = False
		(x0, y0, t0) = self._getOdometry()
		time0 = time.time() #Guardo el instante de comienzo de la funcion bloqueante

		while(not finish):
			v = Vel if pos > 0 else -Vel
			self.move(v, 0)
			time.sleep(self._odometry_interval)

			(xf, yf, tf) = self._getOdometry()
			dif = xf - x0
			timef = time.time() #guardo el instante actual en cada vuelta del bucle

			pos_alcanzada = self._posicion_alcanzada(dif, pos, ErrorRange)
			tout_excedido = ((timef - time0) >= Timeout)

			if(pos_alcanzada or tout_excedido):		
				if(tout_excedido):
					if 10 in DEBUG:
						print("[PiBot] No se pudo alcanzar la posicion deseada", pos)
				finish = True

		self.parar()
		time.sleep(0.12)


	def _alcanzarposicion(self, angle, angle_aux, ErrorRange):
		finish = False
		timeout_exceded = False

		(x0, y0, t0) = self._getOdometry()
		time0 = time.time()
		while(not finish):
			a = 1 if angle > 0 else -1
			self.move(0, a)
			time.sleep(self._odometry_interval)

			(xf, yf, tf) = self._getOdometry()
			dif = tf - t0
			timef = time.time()

			pos_alcanzada = self._posicion_alcanzada(dif, angle_aux, ErrorRange)
			tout_excedido = ((timef - time0) >= Timeout)

			if(pos_alcanzada or tout_excedido):
				finish = True
				
			if(tout_excedido):
				if 10 in DEBUG:
					print("[PiBot] No se pudo alcanzar el ángulo deseado", angle)
				timeout_exceded = True

		return timeout_exceded


	def girarHasta(self, angle):
		'''
		Gira el robot un angulo determinado en sentido horario o antihorario dependiendo
		del signo de 'angle'
		'''
		ErrorRange = 0.2
		Timeout = 10

		nvueltas = abs(angle / (math.pi / 2))
		parte_entera = int(nvueltas) #Son las medias vueltas que tiene que dar
		parte_decimal = abs(nvueltas - parte_entera) #Parte de media vuelta que tendra que dar

		if(nvueltas > 1):
			#Doy todas las medias vueltas que tenga que dar
			for i in range(0, parte_entera):
				if(angle > 0):
					angle_aux = math.pi / 2
				else:
					angle_aux = (-1) * (math.pi /2)
				tout_exceeded = self._alcanzarposicion(angle, angle_aux, ErrorRange)

			#Si me queda una parte de media vuelta por dar, la doy
			if(parte_decimal != 0 and not tout_exceeded):
				if(angle > 0):
					angle_aux = (math.pi / 2) * parte_decimal
				else:
					angle_aux = (-1) * (math.pi / 2) * parte_decimal

				self._alcanzarposicion(angle, angle_aux, ErrorRange)
		else:
			self._alcanzarposicion(angle, angle, ErrorRange)

		self.parar()
		time.sleep(0.12)


	def _arco(self, v, w, x, y):

		(x0, y0, t0) = self._getOdometry()
		finish = False

		while(not finish):
			self.move(v, w)
			time.sleep(self._odometry_interval)

			(xf, yf, tf) = self._getOdometry()
			difx = xf - x0
			dify = yf - y0

			if self._posicion_alcanzada(difx, x, ErrorRangeLinear) or \
				self._posicion_alcanzada(dify, y, ErrorRangeLinear):
				finish = True
		
		return finish


	def arcoHasta(self, x, y, theta):
		'''
		Mueve el robot a la posicion relativa (x, y, theta)
		¡Ojo! Tengo en cuenta que el eje Y aumenta hacia la izquierda
		'''
		#Fijo como constante la velocidad lineal [m/s]
		VelV = 0.03
		#Fijo el radio minimo que se puede trazar [m]
		Rmax = 0.25 - VelV #0.25 es la velocidad maxima en m/s que puede alcanzar un motor

		ErrorRangeLinear = 0.01 #Margen de error para el movimiento

		#Si el movimiento solo es en una componente, llamo a 'moverHasta'
		if(x == 0 and y != 0):
			a = (-math.pi/2) if y > 0 else (math.pi/2)
			self.girarHasta(a)
			self.moverHasta (y)
		elif(x != 0 and y == 0):
			self.moverHasta(x)
		#Si el movimiento es en ambas componentes, he de llamar a 'move'
		else:
			#Defino el radio del arco que tengo que trazar
			r_arco = ((x ** 2) + (y ** 2)) / (2 * y)
			if(abs(r_arco) <= Rmax):
				w = VelV / r_arco
				if((x < 0 and y < 0) or (x < 0 and y > 0)):
					for i in range(0, 2):
						self._arco(VelV, w, x, y)
						self.parar()
						time.sleep(0.013)
				else:
					self._arco(VelV, w, x, y)
			else:
				#Me tengo que mover el linea recta
				alfa = math.atan(y / x) #Calculo la fase (angulo de giro)
				if((x < 0 and y < 0) or (x < 0 and y > 0)):
					alfa = alfa + math.pi

				self.girarHasta(alfa) #Llevo a cabo el giro
				dist = math.sqrt((x ** 2) + (y ** 2)) #Calculo el modulo del vector
				self.moverHasta(dist)
		self.parar()
		time.sleep(0.15)
		#Por ultimo, giro el angulo 'theta' especificado
		self.girarHasta(theta)
		self.parar()
		time.sleep(0.12)


	def move(self, velV, velW):
		'''
		Función que hace avanzar y girar al robot al mismo tiempo, según las velocidades V,W dadas como parámetro.
		@type velV, velW: enteros de [1 a 5, a mas, se corta en 5]
		@param velV, velW: velocidades de avance de motores lineal y angular, en m/s y rad/s respectivamente
		'''

		if(velW != 0):
			rcir = abs(velV / velW) #Es el radio de la circunferencia que tengo que trazar. En valor absoluto
			velmotorgiro = abs(velV) + rcir #Velocidad a la que tiene que girar el motor encargado del giro del robot
		if(velV == 0 and velW != 0):
			#Motor izquierdo hacia atras y motor derecho hacia adelante a velocidad maxima
			self._movermotordcho(velW / 16)
			self._movermotorizq(-velW / 16)
		elif(velV != 0 and velW == 0):
			#Avanza hacia el frente a la velocidad lineal dada
			self._movermotordcho(velV)
			self._movermotorizq(velV)
		elif(velV > 0 and velW > 0):
			self._movermotorizq(velV)
			self._movermotordcho(velmotorgiro)
		elif(velV > 0 and velW < 0):
			self._movermotorizq(velmotorgiro)
			self._movermotordcho(velV)
		elif(velV < 0 and velW > 0):
			self._movermotorizq(-velmotorgiro)
			self._movermotordcho(velV)
		elif(velV < 0 and velW < 0):
			self._movermotorizq(velV)
			self._movermotordcho(-velmotorgiro)


	def leerIRSigueLineas(self):
		'''
		devuelve el estado de los sensores IR
		'''
		right_sensor_port = 22
		left_sensor_port = 27

		self._GPIO.setmode(self._GPIO.BCM)
		self._GPIO.setup(right_sensor_port, self._GPIO.IN)
		self._GPIO.setup(left_sensor_port, self._GPIO.IN)
		right_value = self._GPIO.input(right_sensor_port)
		left_value = self._GPIO.input(left_sensor_port)
		#0: ambos sobre la linea
		#1: izquierdo sobre la linea
		#2: derecho sobre la linea
		#3: ambos fuera de la linea

		if(right_value == 1 and left_value == 1):
			state = 0
		elif(right_value == 0 and left_value == 1):
			state = 1
		elif(right_value == 1 and left_value == 0):
			state = 2
		else:
			state = 3

		return state


	def leerUltrasonido(self):
		'''
		devuelve la distancia a un objeto en metros
		'''

		inp = 3
		out = 2

		self._GPIO.setwarnings(False)
		self._GPIO.setmode(self._GPIO.BCM)
		self._GPIO.setup(out, self._GPIO.OUT)
		self._GPIO.setup(inp, self._GPIO.IN)

		self._GPIO.output(out, False)
		time.sleep(0.00001)
		self._GPIO.output(out, True)
		time.sleep(0.00001)
		self._GPIO.output(out, False)
		start = time.time()
		while(self._GPIO.input(inp) == 0):
		    start = time.time()
		while(self._GPIO.input(inp) == 1):
		    stop = time.time()
		elapsed = stop - start

		return (elapsed * 343) / 2


	def dameImagen (self, resize=None):
		'''
		devuelve una matriz que se corresponde con una imagen
		'''
		self._frame = self._videostream.read()
		if resize:
			self._frame = imutils.resize(self._frame, width=resize)

		return self._frame


	def mostrarImagen (self):
		cv2.imshow("Imagen", self._frame)


	def dameObjeto(self, lower=ORANGE_MIN, upper=ORANGE_MAX, showImageFiltered=False):
		'''
		Función que devuelve el centro del objeto que tiene un color verde en el rango [GREEN_MIN, GREEN_MAX] para ser detectado
		'''
		# resize the image
		image = self.dameImagen()

		# convert to the HSV color space
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

		# construct a mask for the color specified
		# then perform a series of dilations and erosions
		# to remove any small blobs left in the mask
		mask = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		# find contours in the mask and
		# initialize the current center
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None

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
			return center, area, image, mask

		return center, area


	def dameSonarVisual (self):
		'''
		Función que devuelve el array de puntos [X,Y] (Z=0) correspondiente al obstáculo detectado
		'''
		cameraModel = loadPiCamCameraModel ()
		puntosFrontera = 0
		fronteraArray = numpy.zeros((ANCHO_IMAGEN,2), dtype = "float64")

		image = dameImagen ()
		hsvImg = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
		bnImg = cv2.inRange(hsvImg, GREEN_MIN, GREEN_MAX)

		pixel = Punto2D()
		pixelOnGround3D = Punto3D()
		tmp2d = Punto2D()
		puntosFrontera = 0
		j = 0

		# Ground image: recorremos la imagen de abajo a arriba (i=filas) y de izquierda a derecha (j=columnas)
		while (j < ANCHO_IMAGEN): # recorrido en columnas
			i = LARGO_IMAGEN-1
			esFrontera = None
			while ((i>=0) and (esFrontera == None)): # recorrido en filas
				pos = i*ANCHO_IMAGEN+j # posicion actual

				pix = bnImg[i, j] # value 0 or 255 (frontera)

				if (pix != 0): # si no soy negro
					esFrontera = True # lo damos por frontera en un principio, luego veremos
					# Calculo los demás vecinos, para ver de qué color son...
					c = j - 1
					row = i
					v1 = row*ANCHO_IMAGEN+c

					if (not((c >= 0) and (c < ANCHO_IMAGEN) and
					(row >= 0) and (row < LARGO_IMAGEN))): # si no es válido ponemos sus valores a 0
						pix = 0
					else:
						pix = bnImg[row, c]

					if (esFrontera == True): # si NO SOY COLOR CAMPO y alguno de los vecinitos ES color campo...
						pixel.x = j
						pixel.y = i
						pixel.h = 1

						# obtenemos su backproject e intersección con plano Z en 3D
						pixelOnGround3D = getIntersectionZ (pixel, cameraModel)

						# vamos guardando estos puntos frontera 3D para luego dibujarlos con PyGame
						fronteraArray[puntosFrontera][0] = pixelOnGround3D.x
						fronteraArray[puntosFrontera][1] = pixelOnGround3D.y
						puntosFrontera = puntosFrontera + 1
						#print "Hay frontera en pixel [",i,",",j,"] que intersecta al suelo en [",pixelOnGround3D.x,",",pixelOnGround3D.y,",",pixelOnGround3D.z,"]"

				i = i - 1
			j = j + 5

		return fronteraArray


	def _movermotorizq(self, vel):
		'''
		Mueve el motor izquierdo a la velicidad dada como parametro
		'''
		# Puerto de datos para servo izquierdo
		puertoL = 4

		if(vel > 0):

			if(vel == 0):
				self._dit.set_servo_pulsewidth(puertoL, 1520)
			elif(vel <= 0.0355): #velocidad muy lenta
				self._dit.set_servo_pulsewidth(puertoL, 1537) #hacia el frente
			elif(vel > 0.0355 and vel <= 0.0655):
				self._dit.set_servo_pulsewidth(puertoL, 1557)
			elif(vel > 0.0655 and vel <= 0.0925):
				self._dit.set_servo_pulsewidth(puertoL, 1582) #1582
			elif(vel > 0.0925 and vel <= 0.13):
				self._dit.set_servo_pulsewidth(puertoL, 1602)
			elif(vel > 0.13 and vel <= 0.2):
				self._dit.set_servo_pulsewidth(puertoL, 1647)
			else: #velocidad muy rapida
				self._dit.set_servo_pulsewidth(puertoL, 2500)
		else:
			if(vel >= -0.0355): #velocidad muy lenta
				self._dit.set_servo_pulsewidth(puertoL, 1450)
			elif(vel < -0.0355 and vel >= -0.0655):
				self._dit.set_servo_pulsewidth(puertoL, 1430)
			elif(vel < -0.0655 and vel >= -0.0925):
				self._dit.set_servo_pulsewidth(puertoL, 1410) #1410
			elif(vel < -0.0925 and vel >= -0.13):
				self._dit.set_servo_pulsewidth(puertoL, 1390)
			elif(vel < -0.13 and vel >= -0.2):
				self._dit.set_servo_pulsewidth(puertoL, 1345)
			else: #velocidad muy rapida
				self._dit.set_servo_pulsewidth(puertoL, 500)


	def _movermotordcho(self, vel):
		'''
		Mueve el motor derecho a la velicidad dada como parametro
		'''
		# Puerto de datos para servo Derecho
		puertoR = 18

		if(vel > 0):

			if(vel == 0):
				self._dit.set_servo_pulsewidth(puertoR, 1515)
			if(vel <= 0.0355): #velocidad muy lenta
				self._dit.set_servo_pulsewidth(puertoR, 1450) #hacia el frente
			elif(vel > 0.0355 and vel <= 0.0655):
				self._dit.set_servo_pulsewidth(puertoR, 1430)
			elif(vel > 0.0655 and vel <= 0.0925):
				self._dit.set_servo_pulsewidth(puertoR, 1410)
			elif(vel > 0.0925 and vel <= 0.13):
				self._dit.set_servo_pulsewidth(puertoR, 1390) #1390
			elif(vel > 0.13 and vel <= 0.2):
				self._dit.set_servo_pulsewidth(puertoR, 1345)
			else: #velocidad muy rapida
				self._dit.set_servo_pulsewidth(puertoR, 1275) #1290
		else:
			if(vel >= -0.0355): #velocidad muy lenta
				self._dit.set_servo_pulsewidth(puertoR, 1542)
			elif(vel < -0.0355 and vel >= -0.0655):
				self._dit.set_servo_pulsewidth(puertoR, 1562)
			elif(vel < -0.0655 and vel >= -0.0925):
				self._dit.set_servo_pulsewidth(puertoR, 1582)
			elif(vel < -0.0925 and vel >= -0.13):
				self._dit.set_servo_pulsewidth(puertoR, 1603)
			elif(vel < -0.13 and vel >= -0.2):
				self._dit.set_servo_pulsewidth(puertoR, 1640)
			else: #velocidad muy rapida
				self._dit.set_servo_pulsewidth(puertoR, 1700)
	

	@property
	def tipo(self):
		return self._tipo


	@tipo.setter
	def tipo(self, valor):
		self._tipo = valor


	def quienSoy(self):
		print ("Buenos días, humano ¡Soy el Robot PiBot!")


									################################################
									################	ODOMETRÍA	################
									################################################

import threading
import pigpio # Libreria para manejar los servos

class Angles:
	def __init__(self):
		prev = 0
		actual = 0
		dif = 0

class Odometry:

	def __init__(self, d, r, gpio):
        
		if 2 in DEBUG:
			print("[PiBot] Generando odometria...")
			print("\t[Odometría] Radio rueda:",r)
			print("\t[Odometría] Distancia eje:",d)
		
		self._GPIO = gpio
		
		self.lock = threading.Lock()
		#Variables de la posicion relativa del PiBot
		self.posx = 0 #posicion en x
		self.posy = 0 #posicion en y
		self.postheta = 0 #posicion en theta (angular)

        #Creo el objeto que me servira para matar el hilo 'self.hilo'
		self.kill_event = threading.Event()

		#Creo un nuevo hilo que ejecutara el metodo 'readOdometry'
		self.hilo = threading.Thread(target=self.readOdometry, args=(self.kill_event,d,r))
		self.hilo.start()

	
	def stopOdometry(self):

		if 11 in DEBUG:
			print("[Odometría] Cerrando odometría...", end="")
		self.kill_event.set()

		if 11 in DEBUG:
			print("HECHO")


	def getOdometry(self):

		self.lock.acquire()
		aux1 = self.posx
		aux2 = self.posy
		aux3 = self.postheta
		self.lock.release()
		return (aux1, aux2, aux3)
	

	def readOdometry(self, kill_event,d,r):
		'''
		Esta leyendo los angulos de ambos motores llamando a la funcion 'leerangulorueda'
		y va modificando las variables posx, posy, postheta
		'''
		#config = yaml.load(open('config.yml'))
		#Constantes:
		FullCircle = 2 * math.pi #radianes
		#DistRuedas = float(config['Dist_Ruedas']) #metros
		#Rrueda = float(config['Radio_Rueda']) #Radio de la rueda

		DistRuedas = d
		Rrueda = r

		#Pines de los encoders:
		EncoderL = 6
		EncoderR = 24
		sample_time = 0.065 #Intervalo entre mediciones

		#Creo los dos ojetos de la clase Angles
		angleizq = Angles()
		angledcho = Angles()

		(angleizq.prev, angledcho.prev) = self._leerangulos(EncoderL, EncoderR) #Leo el angulo previo
		time.sleep(sample_time)

		while(not kill_event.is_set()):


			(angleizq.actual, angledcho.actual) = self._leerangulos(EncoderL, EncoderR) #Leo el angulo actual
			(angleizq.dif, angledcho.dif) = self._diferenciaangulos(angleizq, angledcho, FullCircle) #calculo la diferencia de angulos en ambas ruedas

			self._calcularposicion(angleizq.dif, angledcho.dif, Rrueda, DistRuedas) #calculo la nueva posicion y la guardo

			(angleizq.prev, angledcho.prev) = (angleizq.actual, angledcho.actual) #los angulos actuales son los previos para la siguiente vuelta
			time.sleep(sample_time)
		
	
	def _leerangulos(self, l, r):
		'''
		l: Encoder del motor izquierdo
		r: Encoder del motor derecho

		Devuelve el angulo en el que se encuentran los motores izquierdo y derecho
		'''
		return (self._leerangulorueda(l), self._leerangulorueda(r))
	

	def _leerangulorueda(self, Encoder):
		'''
		Encoder: encoder del motor cuyo angulo se quiere conocer

		Devuelve el angulo en el que se encuentra la rueda cuyo pin de feedback se encuentra en el
		pin Encoder en el momento en que se llama a la funcion
		'''
		#Constantes:
		#Pi = 3.1416
		DcMin = 29
		DcMax = 971
		FullCircle = 2 * math.pi #360
		DutyScale = 1000
		Q2Min = FullCircle / 4 #angulo minimo perteneciente al segundo cuadrante
		Q3Max = Q2Min * 3 #angulo maximo perteneciente al tercer cuadrante

		turns = 0

		#se inicializa la configuracion de los pines correspondientes:
		self._GPIO.setmode(self._GPIO.BCM)
		self._GPIO.setup(Encoder, self._GPIO.IN)

		#calculo el angulo inicial
		timeHigh = self._pulse_in(Encoder, 1) #devuelve el tiempo en microsegundos
		timeLow = self._pulse_in(Encoder, 0) #devuelve el tiempo en microsegundos
		timeCycle = timeHigh + timeLow
		dutyCycle = (DutyScale * timeHigh) / timeCycle #calculo el ciclo de trabajo
		angle = (FullCircle - 1) - ((dutyCycle - DcMin) * FullCircle) / (DcMax - DcMin + 1)
		p_angle = angle

		finish = False
		while(not finish):
			timeHigh = self._pulse_in(Encoder, 1) #devuelve el tiempo en microsegundos
			timeLow = self._pulse_in(Encoder, 0) #devuelve el tiempo en microsegundos

			timeCycle = timeHigh + timeLow

			if((timeCycle > 1000) and (timeCycle < 1200)):
				finish = True

		dutyCycle = (DutyScale * timeHigh)/ timeCycle #calculo el ciclo de trabajo

		angle = (FullCircle - 1) - ((dutyCycle - DcMin) * FullCircle) / (DcMax - DcMin + 1)
		if(angle < 0):
			angle = 0
		elif(angle > (FullCircle - 1)):
			angle = FullCircle - 1

		#If transition from quadrant 4 to quadrant 1, increase turns count.
		if((angle < Q2Min) and (p_angle > Q3Max)):
			turns = turns + 1
		#If transition from quadrant 1 to  quadrant 4, decrease turns count.
		elif((p_angle < Q2Min) and (angle > Q3Max)):
			turns = turns - 1

		#Construct the angle measurement from the turns count and current angle value.
		if(turns >= 0):
			angle = (turns * FullCircle) + angle
		elif(turns <  0):
			angle = ((turns + 1) * FullCircle) - (FullCircle - angle)
		#Esto lo hago para que cuando repita la vuelta se ponga a cero de nuevo
		if(angle >= FullCircle):
			angle = angle - FullCircle
			turns = 0
		elif(angle <= -FullCircle):
			angle = angle + FullCircle
			turns = 0

		return angle
	

	def _diferencia(self, actual, previo, fullcircle):
		dif = actual - previo

		if(previo > actual):
			if(abs(dif) >= 5.21): #5.21 se corresponde con el 83% de la circunferencia completa. Lo uso para estimar si ha completado una vuelta o no
				dif = fullcircle - previo + actual
		return dif


	def _diferenciaangulos(self, izq, dcho, fullcircle):
		'''
		Recibe dos objetos de la clase 'Angles' y devuelve la diferencia de angulos
		de las ruedas izquierda y derecha.
		'''
		i = self._diferencia(izq.prev, izq.actual, fullcircle)
		d = self._diferencia(dcho.prev, dcho.actual, fullcircle)

		return (i, d)


	def _calcularposicion(self, difizq, difdcho, rrueda, distruedas):
		'''
		Recibe la diferencia de angulo (en radianes) de cada motor (difizq y difdcho) y
		sobreescribe la posicion relativa
		'''	
		#calculo el diferencial de theta:
		longarcoizq = difizq * rrueda #Hayo la longitud de arco que ha trazado cada una de las ruedas:
		longarcodcho = (-1) * difdcho * rrueda #Por convenio, si avanza hacia adelante, la distancia recorrida es positiva
		dtheta = (longarcodcho - longarcoizq) / distruedas

		#calculo la distancia en linea recta que el robot ha recorrido:
		lizq = difizq * rrueda
		ldcho = (-1) * difdcho * rrueda
		d = (lizq + ldcho) / 2

		#calculo x e y:
		x = d * math.cos(dtheta)
		y = d * math.sin(dtheta)
		self.lock.acquire()
		self.posx = self.posx + x
		self.posy = self.posy + y
		self.postheta  = self.postheta + dtheta
		self.lock.release()


	def _readuntil(self, inp, bit):
		rec = self._GPIO.input(inp)
		if(rec == bit):
			#esperar hasta terminar de leer unos
			while(rec == bit):
				rec = self._GPIO.input(inp)
		#leer ceros hasta que me llegue el primer uno
		if(bit == 1):
			while(rec == 0):
				rec = self._GPIO.input(inp)
			#ahora me acaba de llegar el primer uno despues de los ceros
		else:
			while(rec == 1):
				rec = self._GPIO.input(inp)
			#ahora me acaba de llegar el primer cero despues de los unos	


	def _pulse_in(self, inp, bit):
		'''
		inp: pin digital de entrada
		bit: Nivel que quiero testear. 1(nivel alto) o 0(nivel bajo)

		Devuelve el ancho de pulso (en microsegundos) del estado especificado de la señal (alto o bajo)
		'''
		if(bit != 0 and bit != 1):
			return 0
		else:
			self._readuntil(inp, bit) #leo hasta que me llega ese bit
			start = time.time() #guardo la hora actual
		if(bit == 1):
			self._readuntil(inp, 0) #leo hasta que me llega un bit contrario al anterior
		else:
			self._readuntil(inp, 1)
		finish = time.time()
		elapsed = (finish - start) * 1000000 #tiempo en microsegundos

		return elapsed