#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import numpy as np
from functions import *
from proyecto.msg import PersonConfig2
from sensor_msgs.msg import LaserScan

class Lidar(object):
    def __init__(self):
        # Crear el suscriptor al tópico del LiDAR
        topic = 'scan'
        self.pub = rospy.Subscriber(topic, LaserScan, self.callback)
        self.lid = LaserScan()
        
        # Esperar 1 segundo
        rospy.sleep(1)

    def callback(self, msg):
        # Callback para el suscriptor
        self.lid = msg
        
    def get_med_ang(self):
        """ Retorna los valores de la medición y sus ángulos
            respectivos en el sistema del LiDAR
        """
        # Obtener los rangos medidos
        rangos = self.lid.ranges
        ang = np.linspace(0.0, 2*np.pi, len(rangos))
        lim_r  = self.lid.range_max
        rangos = np.clip(rangos,0,lim_r)
        #med1 = np.roll(rangos,len(rangos)//4)
        #med = np.append(med1[357:359],med1[0:182])
        med = rangos
        #ang = np.append(ang[357:359],ang[0:182])
        return med,ang


# Inicializar el nodo
rospy.init_node('nodo_person_config')

# Declarar del publicador
topic = 'person_config'
pub = rospy.Publisher(topic, PersonConfig2, queue_size=10)

# Objeto que lee el escaneo
lidar = Lidar()

# Creación de una instancia (vacía) del mensaje
person_config_msg = PersonConfig2()

# Asignar los valores de la medicion anterior
med1, ang1 = lidar.get_med_ang()
x = med1*np.cos(ang1)
y = med1*np.sin(ang1)+0.15
ang = np.linspace(np.pi,-np.pi,len(y))
ang = np.linspace(0,2*np.pi,len(y))
med = np.power(x*x+y*y,0.5)
med_0 = np.clip(med,0,3)
pos_person_0 = np.array([0,5,-0.9,0,5,0.8])

# Tiempo de ejecución del bucle (en Hz)
freq = 1000
dt = 1.0/freq
rate = rospy.Rate(freq)
t = 0.0

while not rospy.is_shutdown():

    # Lectura de los valores de medidas y angulos
    med1, ang1 = lidar.get_med_ang()
    x = med1*np.cos(ang1)
    y = med1*np.sin(ang1)+0.15
    ang = np.linspace(np.pi,-np.pi,len(y))
    ang = np.linspace(0,2*np.pi,len(y))
    med = np.power(x*x+y*y,0.5)
    med = np.clip(med,0,3)
    
    # Obtener la medicion filtrada
    med_f = lidar_filter(med,med_0,0.75)
    med_f = moving_average(med_f, 5)
    # Actualizar la medicion anterior
    med_0 = med
    # Obtener la posición de la persona
    pos_person_x, pos_person_y = person_position(med_f, ang)
    #pos_person_0 = pos_person

    # Asignar los valores en el mensaje
    person_config_msg.med          = np.round(med, 3)
    person_config_msg.med_f        = np.round(med_f, 3)
    person_config_msg.ang          = np.round(ang, 3)
    person_config_msg.pos_person_x = np.round(pos_person_x, 3)
    person_config_msg.pos_person_y = np.round(pos_person_y, 3)
    
    t = t + dt
    #break
    # Publicar el mensaje
    pub.publish(person_config_msg)
    
    # Esperar
    rate.sleep()
