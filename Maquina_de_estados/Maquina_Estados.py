#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------------------------------------------------------------------------------------#
import os, time, ctypes

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

os.sys.path.append('dynamixel_functions_py')                # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library


#------------------------Conexion del control y funciones de pygame-------------------------#
import pygame
from pygame.locals import *

pygame.init()
pygame.joystick.init()

#-----------------------------------Ajustes de Dynamixel------------------------------------#

# Elementos de la tabla de control del dynamixel AX-12
P_TORQUE_ENABLE             = 24 # byte del habilitacion del torque
P_GOAL_POSITION_L           = 30 # byte bajo de la posición objetivo
P_GOAL_POSITION_H           = 31 # byte alto de la posición objetivo
P_MOVING_SPEED_L            = 32 # byte bajo de la velocidad de movimiento
P_MOVING_SPEED_H            = 33 # byte alto de la velocidad de movimiento
P_PRESENT_SPEED             = 38 # byte bajo de la velocidad

# Longitud de los bytes de datos
GOAL_POSITION_LEN           = 2  # Indica que la posicion objetivo corresponde a 2 bytes

# Version de protocolo
PROTOCOL_VERSION            = 1

# Ajustes por defecto
COMM_SUCCESS                = 0
NUM_ACTUATOR                = 10            # Número de actuadores
MUESTREO                    = 10            # Número de muestras por segundo
BAUDRATE                    = 1000000       # Tasa de transmision
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Revisar el puerto de transmision que se esta usando
                                                            # Linux: "/dev/ttyUSB0", RPi: "/dev/ttyAMA0"
# Inicializar las estructuras de PortHandler
# Establecer la ruta del puerto
# Obtener métodos y miembros de PortHandlerLinux

port_num = dynamixel.portHandler(DEVICENAME)

# Inicializar las estructuras de PacketHandler
dynamixel.packetHandler()

# Inicializa miembros de la estructura de puntero de datos de paquetesInicia
group_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, P_GOAL_POSITION_L, GOAL_POSITION_LEN)

# Abrir Puerto
if dynamixel.openPort(port_num):
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    print("Press any key to terminate...")
    getch()
    quit()

#----------------Parametros e inicio del codigo principal de maquina de estados-------------#

from info_modulos import Modulos
from info_modulos import registro

modulos=[None]*NUM_ACTUATOR                                         # Inicializamos vector principal
                                                                    # de modulos de n posiciones (vacio) 
for i in range(10):
    modulos[i] = Modulos(i+1)
    modulos[i].vector_pos = [512, 512, 512, 512, 512, 512, 512, 512, 512, 512]


while 1:

    joystick = pygame.joystick.Joystick(0)              # Crea un nuevo objeto Joystick
    joystick.init()                                     # Inicializa el objeto Joystick
    evento = pygame.event.wait()                        # Espera un solo evento de la cola
    
    if joystick.get_button(0):                          # Obtiene el estado actual del boton()
        print ("Ha terminado la teleoperación del Robot")
        break
        #raise SystemExit

    for j in range(10):
        registro(joystick, modulos, (10-j))

    ## función para iniciar un conteo, se busca que la instrucción no se demore más de 100 ms
    t_ini=time.time()

    ## Se recorre el numero de servomotores
    for i in range(NUM_ACTUATOR):

        
        ## Se arma el paquete a enviar al servomotor. Es el envío del comando de posición
        dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), modulos[i].vector_pos[0], GOAL_POSITION_LEN)).value
        #Imprime resultado(dxl_addparam_result)
        if dxl_addparam_result != 1:
            print(dxl_addparam_result)
            print("[ID:%03d] groupSyncWrite addparam failed" % (i+1))
            quit()

    ## Escribir la posicion objetivo para el grupo de motores para transmitir el paquete
    dynamixel.groupSyncWriteTxPacket(group_num)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

    ## Eliminar los parametros almacenados 
    dynamixel.groupSyncWriteClearParam(group_num)

    ## Se finaliza el conteo iniciado antes del proceso de transmisión del paquete de estadosfunción para finalilzar un conteo
    t_fin=time.time()

    while ((t_fin-t_ini)*1000<100):
        t_fin=time.time()
    
for i in range(10):
	print(modulos[i].vector_pos)
	
# Cerrar puerto
dynamixel.closePort(port_num)
