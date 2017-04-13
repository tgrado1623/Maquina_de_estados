#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time, os, ctypes, math
import dynamixel_functions as dynamixel

## Longitud de los bytes de datos
GOAL_POSITION_LEN       = 2  ## Indica que la posicion objetivo corresponde a 2 bytes
## Version de protocolo
PROTOCOL_VERSION        = 1
## Ajustes por defecto
NUM_ACTUATOR            = 10        ## Número de actuadores
MUESTREO                = 10
COMM_SUCCESS            = 0

## Variables globales
MOD_ACTIVOS=[1]*NUM_ACTUATOR      ## Dejar el valor 0 para inactivos y el valor 1 para activos

# ------------------------------------------------------------------------------------------------------------------------------- #

## Funciones
def Choset(amplitud_par,amplitud_impar,desfase,dtheta_dn,dtheta_dt,t_time,n,offset_par,offset_impar):
    theta=(dtheta_dn*n + dtheta_dt*(t_time))
    if (n%2==0):
        return (((offset_par*3.14159)/180) + ((amplitud_par*3.14159)/180)*math.sin((theta*3.14159)/180))*180/3.14159
    else:
        return (((offset_impar*3.14159)/180) + ((amplitud_impar*3.14159)/180)*math.sin(((theta+desfase)*3.14159)/180))*180/3.14159

## Variables locales
ID = list(range(1,NUM_ACTUATOR+1))
tiempo = 1

def giroIzq(port_num, group_num):
    ## Parametros de la ecuacion de Choset (rolling)
    amplitud_par        = 10
    amplitud_impar      = 10
    offset_par          = 0
    offset_impar        = 0 
    desfase             = 45
    dtheta_dn           = 0
    dtheta_dt           = 36

    for t_time in range(tiempo):
        for muestra in range(4):
            t_ini=time.time()
            for i in range(NUM_ACTUATOR):
                angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)
                angulo*=MOD_ACTIVOS[i]
                goalpos = 512 - angulo*3.41
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), int(goalpos), GOAL_POSITION_LEN)).value
                if dxl_addparam_result != 1:
                    quit()

            dynamixel.groupSyncWriteTxPacket(group_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            dynamixel.groupSyncWriteClearParam(group_num)
            t_fin=time.time()
            while ((t_fin-t_ini)*1000<100):
                t_fin=time.time()

    ## Parametros de la ecuacion de Choset (Home)
    amplitud_par        = 0
    amplitud_impar      = 0
    offset_par          = 0
    offset_impar        = 0 
    desfase             = 0
    dtheta_dn           = 0
    dtheta_dt           = 0

    for t_time in range(tiempo):
      for muestra in range(MUESTREO):
            t_ini=time.time()
            for i in range(NUM_ACTUATOR):
                angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)
                angulo*=MOD_ACTIVOS[i]
                goalpos = 512 - angulo*3.41
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), int(goalpos), GOAL_POSITION_LEN)).value
                if dxl_addparam_result != 1:
                    quit()

            dynamixel.groupSyncWriteTxPacket(group_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            dynamixel.groupSyncWriteClearParam(group_num)
            t_fin=time.time()
            while ((t_fin-t_ini)*1000<100):
                t_fin=time.time()

def giroDer(port_num, group_num):
    ## Parametros de la ecuacion de Choset (rolling)
    amplitud_par        = 10
    amplitud_impar      = 10
    offset_par          = 0
    offset_impar        = 0 
    desfase             = 45
    dtheta_dn           = 0
    dtheta_dt           = -36

    for t_time in range(tiempo):
        for muestra in range(2):
            t_ini=time.time()
            for i in range(NUM_ACTUATOR):
                angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)
                angulo*=MOD_ACTIVOS[i]
                goalpos = 512 - angulo*3.41
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), int(goalpos), GOAL_POSITION_LEN)).value
                if dxl_addparam_result != 1:
                    quit()

            dynamixel.groupSyncWriteTxPacket(group_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            dynamixel.groupSyncWriteClearParam(group_num)
            t_fin=time.time()
            while ((t_fin-t_ini)*1000<100):
                t_fin=time.time()

    ## Parametros de la ecuacion de Choset (Home)
    amplitud_par        = 0
    amplitud_impar      = 0
    offset_par          = 0
    offset_impar        = 0 
    desfase             = 0
    dtheta_dn           = 0
    dtheta_dt           = 0

    for t_time in range(tiempo):
      for muestra in range(MUESTREO):
            t_ini=time.time()
            for i in range(NUM_ACTUATOR):
                angulo = Choset(amplitud_par, amplitud_impar, desfase, dtheta_dn, dtheta_dt, (muestra+(10*(t_time))), i, offset_par, offset_impar)
                angulo*=MOD_ACTIVOS[i]
                goalpos = 512 - angulo*3.41
                dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(group_num, (i+1), int(goalpos), GOAL_POSITION_LEN)).value
                if dxl_addparam_result != 1:
                    quit()

            dynamixel.groupSyncWriteTxPacket(group_num)
            if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

            dynamixel.groupSyncWriteClearParam(group_num)
            t_fin=time.time()
            while ((t_fin-t_ini)*1000<100):
                t_fin=time.time()
