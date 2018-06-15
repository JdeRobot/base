PiBot es un robot educacional inventado por el equipo de JdeRobot. Consta de una Raspberry PI, una PiCam, varios sensores y actuadores comunes (y baratos), una bateria y un conjunto de piezas imprimibles.

Es Hardware abierto.

Los drivers son de código abierto, por lo que los estudiantes pueden programar aplicaciones del PiBot con python. Además, Pueden programar para el PiBot simulado en gazebo. El modelo y los plugins tambien usan el mismo interfaz de programacion que el PiBot real.

![Pibot real][PiBot-real]
![Pibot simulado][PiBot-sim]

[PiBot-sim]: http://jderobot.github.io/JdeRobot/pibot-2.png "Simulated PiBot"
[PiBot-real]: http://jderobot.github.io/JdeRobot/pibot-1.jpg "Real PiBot"


# Interfaz de Programación

La capa de abstracción de hardware

La capa de abstracción de hardware (HAL, Hardware Abstraction Layer) está compuesta por 4 tipos: sensores puros, actuadores puros, sensores personalizados y actuadores personalizados.

| Función | Significado |
| ------ |------|
| avanzar | Avanza con la velocidad dada|
| retroceder | Retrocede con la velocidad dada|
| parar | El robot se detiene|
| girarIzquierda | Gira con la velocidad dada |
| girarDerecha | Gira con la velocidad dada |
| leerUltrasonido | Devuelve la distacia leida por el ultrasonido |
| leerIRSigueLineas | Devuelve el valor de los siguelineas (0: ambos sensores sobre la linea, 1: solo sensor izquierdo sobre la linea, 2: solo sensor derecho sobre la linea, 3: ambos sensores fuera de la linea) |
| leerIntensidadSonido | Get US measurement |
| leerPotenciometro | devuelve el valor del potenciometro |
| moverServo | mueve el servo a la posición indicada |
| dameImagen | Devuelve la imagen de la camara del robot |
| damePosicionDeObjetoDeColor | Devuelve la posición del objeto del color indicado en la imagen dada |



# Lista de la compra

* Raspberry Pi 3
* Micro SD de 16GB
* Batería (3A output, 20000mAh)
* 3 motores: como [Parallax Servo Feedback 360º](https://www.parallax.com/product/900-00360)
* PiCam camera
* Sensores IR
* sensor de ultrasonido HC-SR04 


# PiBot en acción

<a href="http://www.youtube.com/watch?feature=player_embedded&v=WUeVjef1p6U" target="_blank"><img src="http://img.youtube.com/vi/WUeVjef1p6U/0.jpg" 
alt="Pibot Real moviéndose" width="240" height="180" border="10" /></a>