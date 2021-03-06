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
        
        # Esperar 1 segundo
        rospy.sleep(1)
        
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
        return ang, med, med_f, pos
        

   
if __name__ == '__main__':
    rospy.init_node("plotter_xy")
    p_conf = PosPerson()
    
    ang, med, med_f, pos_person = p_conf.get_pos_person()
    
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
    
    x = med_f*np.cos(ang) + 0.0
    y = med_f*np.sin(ang) + 0.5    
    
    plt.ion()
    
    figure, ax = plt.subplots(figsize=(10, 8))

    splot = 2
    
    if splot == 1:
        graf, = ax.plot(ang,med,'b')
        graf2, = ax.plot(th1,r1,'*r')
        graf3, = ax.plot(th2,r2,'*r')
        graf4, = ax.plot(ang,med_f,'r')
        plt.axis([0.0, np.pi, 0, 3.5])
    else:
        graf5, = ax.plot(x,y,'b')    
        graf4, = ax.plot(0,1,'*k')
        plt.axis([-4, 4, 0, 4.5])
    
    plt.xlabel('Angulo(rad)')
    plt.ylabel('Distancia medida')
    plt.grid()
    
    freq = 100
    dt = 1.0/freq
    rate = rospy.Rate(freq)
    
    
    while not rospy.is_shutdown():
    
        ang, med, med_f, pos_person = p_conf.get_pos_person()
        
        pos,_ = find_peaks(-med_f,distance=17)
                
        if len(pos) >= 2:
            med_p = med[pos]
            ang_p = ang[pos]
    
        r1 = med_p[0]+0.1
        th1 = ang_p[0]
        r2 = med_p[1]+0.1
        th2 = ang_p[1]
        
        x = med_f*np.cos(ang) + 0.0
        y = med_f*np.sin(ang) + 0.5
        
        
        if splot == 1:
            graf.set_xdata(ang)
            graf.set_ydata(med)
            graf2.set_xdata(th1)
            graf2.set_ydata(r1)
            graf3.set_xdata(th2)
            graf3.set_ydata(r2)
            graf4.set_xdata(ang)
            graf4.set_ydata(med_f)
        else:
            graf4.set_xdata(pos_person[0])
            graf4.set_ydata(pos_person[1])
            graf5.set_xdata(x)
            graf5.set_ydata(y)
        
        figure.canvas.draw()
        figure.canvas.flush_events()
 
        time.sleep(0.1)
            
        #rate.sleep()

"""

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
