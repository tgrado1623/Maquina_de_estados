import pygame

class Modulos(object):

    def __init__(self,ID):
        self.vector_pos=[]
        self.ID_modulo=ID
        if (self.ID_modulo%2==0):
            self.tipo="par"
        else:
            self.tipo="impar"


def registro (joystick, vector, ID):
	
    # "Left_Analog x": "axis 0",
    # "Right_Analog y": "axis 3",
    if (vector[ID-1].ID_modulo==1):
        if (len(vector[ID-1].vector_pos)>=10):
            for i in range(len(vector[ID-1].vector_pos)-9):
                vector[ID-1].vector_pos.pop(-1)
            vector[ID-1].vector_pos.insert(0,int(512-(joystick.get_axis(0)**3)*154))

    if (vector[ID-1].ID_modulo==2):
        if (len(vector[ID-1].vector_pos)>=10):
            for i in range(len(vector[ID-1].vector_pos)-9):
                vector[ID-1].vector_pos.pop(-1)
            vector[ID-1].vector_pos.insert(0,int(512-(joystick.get_axis(3)**3)*154))

    # "L2": "axis 12",
    # "R2": "axis 13",
    if ((vector[ID-1].tipo=="impar") and (vector[ID-1].ID_modulo!=1)):
        retardo_impar = 10-int(10*(joystick.get_axis(13)))
        if retardo_impar==10:
            retardo_impar=9
        print("Retardo impar: ",retardo_impar)
        if (len(vector[ID-1].vector_pos)>=10):
            for i in range(len(vector[ID-1].vector_pos)-9):
                vector[ID-1].vector_pos.pop(-1)
            vector[ID-1].vector_pos.insert(0,int(((vector[ID-3].vector_pos[retardo_impar])-512)*0.8)+512)

    if ((vector[ID-1].tipo=="par") and (vector[ID-1].ID_modulo!=2)):
        retardo_par = 10-int(10*(joystick.get_axis(12)))
        if retardo_par==10:
            retardo_par=9
        print("Retardo par: ",retardo_par)
        if (len(vector[ID-1].vector_pos)>=10):
            for i in range(len(vector[ID-1].vector_pos)-9):
                vector[ID-1].vector_pos.pop(-1)
            vector[ID-1].vector_pos.insert(0,int(((vector[ID-3].vector_pos[retardo_par])-512)*0.8)+512)
    
