# -*- coding: utf-8 -*-

from __future__ import print_function

import threading
import pigpio # Libreria para manejar los servos

'''
DEBUG
1: Inicialización driver
2: Inicialización sensores
10: Odometría
11: Finalización 
'''

DEBUG = [10, 11]
#DEBUG = [10]

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