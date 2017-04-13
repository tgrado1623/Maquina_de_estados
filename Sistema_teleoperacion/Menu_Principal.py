#!/usr/bin/env python
# -*- coding: utf-8 -*-

#------------------------Conexion del control y funciones de pygame-------------------------#
import pygame,sys
from pygame.locals import *
from Funcion_orientacion  import giroIzq, giroDer


pygame.init()
pygame.joystick.init()

import dynamixel_functions as dynamixel

## Elementos de la tabla de control del dynamixel AX-12
P_GOAL_POSITION_L       = 30 ## byte bajo de la posición objetivo

## Longitud de los bytes de datos
GOAL_POSITION_LEN       = 2  ## Indica que la posicion objetivo corresponde a 2 bytes

## Version de protocolo
PROTOCOL_VERSION        = 1

## Ajustes por defecto
BAUDRATE                = 1000000   ## Tasa de transmisión
DEVICENAME              = "/dev/ttyUSB0".encode('utf-8') # Revisar el puerto de transmision que se esta usando
                                                         # Linux: "/dev/ttyUSB0", RPi: "/dev/ttyAMA0"




port_num = dynamixel.portHandler(DEVICENAME)
dynamixel.packetHandler()
group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, P_GOAL_POSITION_L, GOAL_POSITION_LEN)
dynamixel.openPort(port_num)
dynamixel.setBaudRate(port_num, BAUDRATE)

while 1:

    joystick = pygame.joystick.Joystick(0)              # Crea un nuevo objeto Joystick
    joystick.init()                                     # Inicializa el objeto Joystick
    evento = pygame.event.wait()                        # Espera un solo evento de la cola

    if joystick.get_button(3):                          # Obtiene el estado actual del boton()
        print ("Ha terminado la teleoperación del Robot")
        break
        #raise SystemExit

    if joystick.get_axis(18):    						# Equis
        import Maquina_Estados
        Maquina_Estados.maquinaEstados(port_num, group_num)

    if joystick.get_axis(17):							# Circulo
        import Marchas_guiadas
        Marchas_guiadas.guiadas(port_num, group_num)

    if joystick.get_axis(16):							# Triangulo
        import Parametrizadas_ps3
        Parametrizadas_ps3.parametrizadas(port_num, group_num)

    if joystick.get_button(7):							# Izquierda
        giroIzq(port_num, group_num)
        

    if joystick.get_button(5):							# Derecha
        giroDer(port_num, group_num)
        
		
dynamixel.closePort(port_num)

