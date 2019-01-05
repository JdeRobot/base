# Introduction
This document is user guide of driver **piBot.py**

First of all, we have to create our object named *robot* in this example. You can
choose the name you want. We do this as follow:
```
import piBot

robot = piBot.PiBot("PiCam")
```

This driver allows you to control all functionalities of your PiBot.

What will be told in the next lines is the specification of each method
of this driver, showing his header and the parameters it receive, with a brief usage example of him.

## 1º *avanzar*
The robot will go forward with a specificated speed.

### Header:
The header of this method ir ``avanzar(vel)``.
### Parameters:
* **vel:** Velocity which you want the robot moves forward in meters per second. Velocity range
is between 0 and 0.25 [m/s].

### Usage Example:
```
try:
  while(True):
    robot.avanzar(0.08)
    time.sleep(0.2)
except KeyboardInterrupt:
  robot.fin()
```
This will do that the robot goes forward with a speed of 0.08 meters per second until
user press ``ctrl+c``; then, the robot will stop. Later is explained the method *fin*.
## 2º *retroceder*
The robot will go back with a specificated speed.

### Header:
His header is ``retroceder(self, vel)``.
### Parameters:
* **vel:** Velocoty which you want the robot moves backward in meters per second.
Velocity range is between 0 and 0.25 [m/s].

### Usage Example:
```
try:
  while(True):
    robot.retroceder(0.15)
    time.sleep(0.2)
except KeyboardInterrupt:
  robot.fin()
```
It will do that the robot goes backward with a sèed of 0.15 meters per second until user press ``crtl+c``; then, the robot will stop.

## 3º *parar*
It will do that robot stops.

### Header:
His header is ``parar()``.

### Parameters:
As you can see, this method does not receive parameters.

### Usage Example:
```
while(True):
  robot.avanzar(0.05)
  time.sleep(5)
  robot.parar()
  time.sleep(2)
  robot.retroceder(0.05)
  time.sleep(5)
  robot.parar()
  time.sleep(2)
```
As you can observe, this code will do that robot goes forward during 5 seconds, later
it will stop during 2 seconds, then robot will go backward during 5 seconds and it will
stop during 2 sencond again. That secuence will be executing in an infinite loop.

## 4º *girarIzquierda*
This method will do that robot turns left.

### Header:
His header is ``girarIzquierda()``.

### Parameters:
This method does not receive parameters.

### Usage Example:
```
robot.girarIzquierda()
time.sleep(1.5)
```
This code serves to do turn left the robot during 1.5 seconds.

## 5º *girarDerecha*
His functionlity is  to turn right the robot.

### Header:
His header is ``girarDerecha()``.

### Parameters:
This method does not receive parameters.

### Usage Example:
```
robot.girarDerecha()
time.sleep(1.5)
```
This code serves to do turn right the robot during 1.5 seconds.

## 6º *moverHasta*
This method serves to do robot to describe a linear movement to a
specificated position.

### Header:
His header is ``moverHasta(pos)``.

### Parameters:
* **pos:** Position which you want the robot moves, in meters. If *pos* has a positive value, robot will move forward; if *pos* has a negative value, robot will move backward.

### Usage Example:
```
robot.moverHasta(0.5)
robot.parar()
```
This code will do robot to go forward 0.5 meters and them, it stop.

## 7º *girarHasta*
This method serves to do robot to turn an specificated angle.

### Header:
His header is ``girarHasta(angle)``.

### Parameters:
* **angle:** This is the angle, in radians, that robot has to turn. If angle value is positive, the robot will turn left; if angle value is negative, the robot will turn right.

### Usage Example:
```
Pi = 3.1416
robot.girarHasta(Pi / 2)
robot.girarHasta(-Pi)
```
This code will do robot to turn left 90 degrees (Pi / 2 radians) and after, it turn right 180 degrees (Pi radians).

## 8º *arcoHasta*
*arcoHasta* allows you to move the robot to a specific location. The location in the plane is as follow:

On one hand, there is an "linear" position, given by 'x' position and 'y' position. On the other hand, there is an "angular" position, given by an angle ("theta", for example).

### Header:
The header of this method is `arcoHasta(x, y, theta)`.

### Parameters:
* **x:** Position in meters of the "x" axis that you want the robot to move.
* **y:** Position in meters of the "y" axis that you want the robot to move.
* **theta:** Angle in radians that you want the robot turn.

### Usage Example:
```
Pi = 3.1416

x = 0.3
y = 0.4
t = Pi / 2
robot.arcoHasta(x, y, t)
```
The robot will move to the relative position (0.3, 0.4, Pi / 2).

## 9º *move*
This method allows you to move the robot with a speed control. You can move the robot with a specificated linear and angular velocity.

Depending on the the sign of each parameter, the robot will do different arcs in different directions.

### Header:
His header is ``move(velV, velW)``.

### Parameters:
* **velV:** Linear speed, in meters per second, that you want the robot to move.
* **velW:** Angular speed, in radians per second, that you want the robot to move.

### Usage Example:
```
robot.move(0.05, 0.8)
time.sleep(4)
robot.fin()
```
This code will do the robot to move describing an arc countercloskwise with linear speed of 0.05 meters per second and 0.8 radians per second.

## 10º leerIRSiguelineas
This method serves to read the state of the infrared sensors.
PiBot has two infrared sensors and each of them is capable of detect if it es being over a black place. So there is four possible cases and, method *leerIRSigueLineas* returns a different number depending on each possibility.
The number code that this method returns is as follow:

| Left Sensor State | Right Sensor State | Returned Value |
| ----------------- | ------------------ | -------------- |
| 0                 | 0                  | 3              |
| 0                 | 1                  | 2              |
| 1                 | 0                  | 1              |
| 1                 | 1                  | 0              |

### Header:
His header is ``leerIRSigueLineas()``.

### Parameters:
This method does receive any parameter.

### Usage Example
```
try:

  while(True):
    if(robot.leerIRSigueLineas() == 1):
      robot.avanzar(0.05)
    else:
      robot.girarDerecha()
    time.sleep(0.1)

except KeyboardInterrupt:
  robot.fin()
```
This code will do the robot to go forward while robot is over the black line. If not, robot will turn right.

## 11º leerUltrasonido
It returns the distance (in meters) between robot and an object.

### Header:
His header is ``leerUltrasonido``.

### Parameters:
This method does receive any parameter.

### Usage Example:
```
try:

  while(True):
    dist = robot.leerUltrasonido()
    print(dist)
    if(dist > 0.15)
      robot.avanzar(0.08)
    else:
      girarHasta(Pi / 2)

except KeyboardInterrupt:
  pass
```
This code will do the robot to go forward while there is not an object nearer than 15 cm (0.15 m), and else, turn left 90 degrees (Pi / 2). In addition, it will we printed on the screen the different distances that sensor reads.
