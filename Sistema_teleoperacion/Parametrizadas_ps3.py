#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------ #
import time, os, ctypes, math
# ------------------------------------------------------------------------------ #
from Funcion_orientacion  import giroIzq, giroDer
# ------------------------------------------------------------------------------ #
import dynamixel_functions as dynamixel
# -------------------Conexion del control y funciones de pygame----------------- #
import pygame
from pygame.locals import *
pygame.init()
pygame.joystick.init()
#------------------------------------------------------------------------------- #

## Longitud de los bytes de datos
GOAL_POSITION_LEN       = 2  ## Indica que la posicion objetivo corresponde a 2 bytes
## Version de protocolo
PROTOCOL_VERSION        = 1
## Ajustes por defecto
NUM_ACTUATOR            = 10        ## Número de actuadores
COMM_SUCCESS            = 0
## Variables globales
MOD_ACTIVOS=[1]*NUM_ACTUATOR      ## Dejar el valor 0 para inactivos y el valor 1 para activos


## Funciones
def Choset(amplitud_par,amplitud_impar,desfase,dtheta_dn,dtheta_dt,t_time,n,offset_par,offset_impar):
    theta=(dtheta_dn*n + dtheta_dt*(t_time))
    if (n%2==0):
        return (((offset_par*3.14159)/180) + ((amplitud_par*3.14159)/180)*math.sin((theta*3.14159)/180))*180/3.14159
    else:
        return (((offset_impar*3.14159)/180) + ((amplitud_impar*3.14159)/180)*math.sin(((theta+desfase)*3.14159)/180))*180/3.14159

def parametrizadas(port_num, group_num):

    ## Variables locales
    ID = list(range(1,NUM_ACTUATOR+1))

    t_time=0
    muestra=0

    amplitud_par        = 0
    amplitud_impar      = 0
    offset_par          = 0
    offset_impar        = 0
    desfase             = 0
    dtheta_dn           = 0
    dtheta_dt           = 0
    
    while 1:

        joystick = pygame.joystick.Joystick(0)              # Crea un nuevo objeto Joystick
        joystick.init()                                     # Inicializa el objeto Joystick
        evento = pygame.event.wait()                        # Espera un solo evento de la cola

        if joystick.get_button(0):                          # Obtiene el estado actual del boton()
            print ("Ha terminado la teleoperación del Robot")            
            break
            #raise SystemExit

        if(joystick.get_axis(18)):
            # Home
            amplitud_par        = 0
            amplitud_impar      = 0
            offset_par          = 0
            offset_impar        = 0
            desfase             = 0
            dtheta_dn           = 0
            dtheta_dt           = 0

        elif(joystick.get_axis(16)):
            # Linear
            amplitud_par        = 0
            amplitud_impar      = 30
            offset_par          = 0
            offset_impar        = 0
            desfase             = 0
            dtheta_dn           = 60
            dtheta_dt           = 36

        elif(joystick.get_axis(19)):
            # Linear_high
            amplitud_par        = 0
            amplitud_impar      = 45
            offset_par          = 10
            offset_impar        = 0
            desfase             = 0
            dtheta_dn           = 60
            dtheta_dt           = 18

        elif(joystick.get_axis(17)):
            # Linear_offset
            amplitud_par        = 0
            amplitud_impar      = 30
            offset_par          = 10
            offset_impar        = 0
            desfase             = 0
            dtheta_dn           = 60
            dtheta_dt           = 36

        elif(joystick.get_axis(13)):
            # Rolling
            amplitud_par        = 10
            amplitud_impar      = 10
            offset_par          = 0
            offset_impar        = 0
            desfase             = 45
            dtheta_dn           = 0
            dtheta_dt           = 36

        elif(joystick.get_axis(12)):
            # Sidewinding
            amplitud_par        = 10
            amplitud_impar      = 40
            offset_par          = 0
            offset_impar        = 0
            desfase             = 45
            dtheta_dn           = 30
            dtheta_dt           = 36
    
        elif(joystick.get_axis(14)):
            # Sidewinding_low
            amplitud_par        = 10
            amplitud_impar      = 40
            offset_par          = 0
            offset_impar        = 0
            desfase             = 45
            dtheta_dn           = 60
            dtheta_dt           = 36

        elif(joystick.get_axis(15)):
            # Helix_rolling
            amplitud_par        = 60
            amplitud_impar      = 60
            offset_par          = 0
            offset_impar        = 0
            desfase             = 90
            dtheta_dn           = 10
            dtheta_dt           = 18

        else:
            dtheta_dt           = 0
            t_time              = 0
        
        if joystick.get_button(7):				# Izquierda
            giroIzq(port_num, group_num)

        if joystick.get_button(5):				# Derecha
            giroDer(port_num, group_num)

        ## función para iniciar un conteo, se busca que la instrucción no se demore más de 100 ms
        t_ini=time.time()

        ## Se recorre el numero de servomotores
        for i in range(NUM_ACTUATOR):

            ## Se calcula el ángulo para el número de servomotor en el tiempo determinado
            angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)

            ## Si el módulo se seleccionó como inactivo, el ángulo se vuelve cero.
            angulo*=MOD_ACTIVOS[i]
            goalpos = 512 - angulo*3.41
            #print(goalpos)

            ## Se arma el paquete a enviar al servomotor. Es el envío del comando de posición
            ## Add Dynamixel#1 goal position value to the Syncwrite storage
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), int(goalpos), GOAL_POSITION_LEN)).value
            #print(dxl_addparam_result)
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (i+1))
                quit()

        ## Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(group_num)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        ## Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(group_num)

        ## Se finaliza el conteo iniciado antes del proceso de transmisión del paquete de estadosfunción para finalilzar un conteo
        t_fin=time.time()

        while ((t_fin-t_ini)*1000<100):
            t_fin=time.time()

        muestra+=1
	
        if (muestra%10==0):
            t_time+=1
            muestra=0
