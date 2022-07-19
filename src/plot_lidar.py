#!/usr/bin/env python 

import numpy as np
from matplotlib import pyplot as plt
import warnings
from scipy.signal import find_peaks
import time
warnings.filterwarnings("ignore",".*GUI is implemented.*")

import rospy
from proyecto.msg import PersonConfig


class PosPerson(object):
    def __init__(self):
        topic    = 'person_config'
        self.pub = rospy.Subscriber(topic, PersonConfig, self.callback)
        self.pperson = PersonConfig()
        
    def callback(self, msg):
        self.pperson = msg
        
    def get_pos_person(self):
        # Obtener la posicion de la capsula de la persona en el
        # espacio cartesiano
        ang = self.pperson.ang
        ang = np.array(ang)
        med = self.pperson.med
        med = np.array(med)
        med_f = self.pperson.med_f
        med_f = np.array(med_f)
        pos = self.pperson.pos_person
        pos = np.array(pos)     
        if len(pos) < 6:
            u = 0
        else:
            u = 1 
        return ang, med, med_f, pos,u
        

   
if __name__ == '__main__':
    rospy.init_node("plotter_xy")
    p_conf = PosPerson()
    
    ang, med, med_f, pos_person,u = p_conf.get_pos_person()
    
    pos,_ = find_peaks(-med,distance=10)
    
    if len(pos) >= 2:
        med_p = med[pos]
        ang_p = ang[pos]
    med_p = np.array([10,10])
    ang_p = np.array([0, 0])
    r1 = med_p[0]+0.05
    th1 = ang_p[0]
    r2 = med_p[1]+0.05
    th2 = ang_p[1]
        
        
    
    plt.ion()
    
    figure, ax = plt.subplots(figsize=(10, 8))
    
    graf, = ax.plot(ang,med,'.b')
    graf2, = ax.plot(th1,r1,'*r')
    graf3, = ax.plot(th2,r2,'*r')
    graf4, = ax.plot(0,1,'*k')
    
    plt.xlabel('Angulo(rad)')
    plt.ylabel('Distancia medida')
    plt.grid()
    plt.axis([0.0, np.pi, 0, 3.5])
    
    freq = 20
    dt = 1.0/freq
    rate = rospy.Rate(freq)
    
    
    while not rospy.is_shutdown():
    
        ang, med, med_f, pos_person,u = p_conf.get_pos_person()
        
        pos,_ = find_peaks(-med_f,distance=17)
                
        if len(pos) >= 2:
            med_p = med[pos]
            ang_p = ang[pos]
    
        r1 = med_p[0]+0.1
        th1 = ang_p[0]
        r2 = med_p[1]+0.1
        th2 = ang_p[1]
        
        graf.set_xdata(ang)
        graf.set_ydata(med)
        graf2.set_xdata(th1)
        graf2.set_ydata(r1)
        graf3.set_xdata(th2)
        graf3.set_ydata(r2)
        graf4.set_xdata(pos_person[0])
        graf4.set_ydata(pos_person[1])
        
        figure.canvas.draw()
        figure.canvas.flush_events()
 
        time.sleep(0.1)
            
        #rate.sleep()

"""
ang= [0.0, 0.018, 0.035, 0.053, 0.07, 0.088, 0.105, 0.123, 0.14, 0.158, 0.176, 0.193, 0.211, 0.228, 0.246, 0.263, 0.281, 0.298, 0.316, 0.333, 0.351, 0.369, 0.386, 0.404, 0.421, 0.439, 0.456, 0.474, 0.491, 0.509, 0.527, 0.544, 0.562, 0.579, 0.597, 0.614, 0.632, 0.649, 0.667, 0.684, 0.702, 0.72, 0.737, 0.755, 0.772, 0.79, 0.807, 0.825, 0.842, 0.86, 0.878, 0.895, 0.913, 0.93, 0.948, 0.965, 0.983, 1.0, 1.018, 1.035, 1.053, 1.071, 1.088, 1.106, 1.123, 1.141, 1.158, 1.176, 1.193, 1.211, 1.229, 1.246, 1.264, 1.281, 1.299, 1.316, 1.334, 1.351, 1.369, 1.387, 1.404, 1.422, 1.439, 1.457, 1.474, 1.492, 1.509, 1.527, 1.544, 1.562, 1.58, 1.597, 1.615, 1.632, 1.65, 1.667, 1.685, 1.702, 1.72, 1.738, 1.755, 1.773, 1.79, 1.808, 1.825, 1.843, 1.86, 1.878, 1.895, 1.913, 1.931, 1.948, 1.966, 1.983, 2.001, 2.018, 2.036, 2.053, 2.071, 2.089, 2.106, 2.124, 2.141, 2.159, 2.176, 2.194, 2.211, 2.229, 2.247, 2.264, 2.282, 2.299, 2.317, 2.334, 2.352, 2.369, 2.387, 2.404, 2.422, 2.44, 2.457, 2.475, 2.492, 2.51, 2.527, 2.545, 2.562, 2.58, 2.598, 2.615, 2.633, 2.65, 2.668, 2.685, 2.703, 2.72, 2.738, 2.755, 2.773, 2.791, 2.808, 2.826, 2.843, 2.861, 2.878, 2.896, 2.913, 2.931, 2.949, 2.966, 2.984, 3.001, 3.019, 3.036, 3.054, 3.071, 3.089, 3.106, 3.124, 3.142]
med= [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.172, 1.122, 1.114, 1.088, 1.074, 1.075, 1.058, 1.072, 1.077, 1.073, 1.089, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.875, 0.854, 0.839, 0.847, 0.85, 0.827, 0.85, 0.85, 0.853, 0.887, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
med_f= [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.17, 1.126, 1.11, 1.086, 1.073, 1.076, 1.052, 1.068, 1.076, 1.073, 1.088, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.874, 0.852, 0.841, 0.844, 0.843, 0.831, 0.848, 0.847, 0.855, 0.887, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
pos_person= [-0.036, 1.47, 0.3, -0.036, 1.47, 2.0]





x = med_f*np.cos(ang) + 0.0
y = med_f*np.sin(ang) + 0.5
print(len(ang))
plt.subplot(121)
plt.plot(ang,med, 'b',ang,med_f,'r--')
plt.xlabel('Angulo(rad)')
plt.ylabel('Distancia medida')
plt.grid()
plt.subplot(122)
plt.plot(x,y,'b',pos_person[0],pos_person[1],'r*')
plt.xlabel('Posicion X')
plt.ylabel('Posicion Y')
plt.grid()
plt.show()
"""
