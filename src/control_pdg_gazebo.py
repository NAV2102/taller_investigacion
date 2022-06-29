#!/usr/bin/env python

import os
import sys

if 'ROS_NAMESPACE' not in os.environ:
    # Default namespace if not set
    # Note, didn't need to check sys.argv for `__ns:=...`,
    # it seems __ns:= takes precidence over ROS_NAMESPACE

    os.environ['ROS_NAMESPACE'] = '/first_robot/arm_controller'

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from functions import *
from roslib import packages

import rbdl
pi = np.pi

if __name__ == '__main__':

    rospy.init_node("control_pdg")

    robot_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
    
    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    # Archivos donde se almacenara los datos
    fqact = open("../texto/qactual.dat", "w")
    fqdes = open("../texto/qdeseado.dat", "w")
    fxact = open("../texto/xactual.dat", "w")
    fxdes = open("../texto/xdeseado.dat", "w")
    fu    = open("../texto/u.dat", "w")

    # Nombres de las articulaciones
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # =============================================================
    # Configuracion articular inicial (en radianes)
    q = np.array([0.0, -1.2, 1.7, -2.2, -1.6, 0.0])
    # Velocidad inicial
    dq = np.array([0., 0., 0., 0., 0., 0.])
    # Configuracion articular deseada
    qdes = np.array([-pi/2, -0.5, 0.5, -pi/2, -pi/2, 0.0])
    # =============================================================
    # Posicion resultante de la configuracion articular deseada
    xdes = ur5_fkine(qdes)[0:3,3]
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = jnames
    
    # Initial position
    goal.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=dq,time_from_start=rospy.Duration(2.0))]
    robot_client.send_goal(goal)
    robot_client.wait_for_result()
    rospy.sleep(1)
    
    # Modelo RBDL
    modelo = rbdl.loadModel('../urdf/ur5_robot2.urdf')
    ndof   = modelo.q_size     # Grados de libertad
    
    # Frecuencia del envio (en Hz)
    freq = 20
    dt = 1.0/freq
    rate = rospy.Rate(freq)
    
    # Simulador dinamico del robot
    robot = Robot(q, dq, ndof, dt)
    
    # Se definen las ganancias del controlador
    Kp = np.diag([8.4, 14.0, 9.1, 6.65, 5.6, 1.75])
    Kd = np.diag([4.08, 6.01, 3.48, 2.52, 3.01, 0.36])
    
    # Bucle de ejecucion continua
    t = 0.0
    while not rospy.is_shutdown():
        robot_client.cancel_goal()
        
        # Leer valores del simulador
        q  = robot.read_joint_positions()
        dq = robot.read_joint_velocities()
        # Posicion actual del efector final
        x = ur5_fkine(q)[0:3,3]
        
        # Almacenamiento de datos
        fxact.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+'\n')
        fxdes.write(str(t)+' '+str(xdes[0])+' '+str(xdes[1])+' '+
                str(xdes[2])+'\n')
        fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+
                ' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n ')
        fqdes.write(str(t)+' '+str(qdes[0])+' '+str(qdes[1])+' '+ str(qdes[2])+
                ' '+ str(qdes[3])+' '+str(qdes[4])+' '+str(qdes[5])+'\n ')

        # ----------------------------
        # Control dinamico (COMPLETAR)
        # ----------------------------
        u = np.zeros(ndof)   # Reemplazar por la ley de control
        g = np.zeros(ndof)
        zeros = np.zeros(ndof)
    
        rbdl.InverseDynamics(modelo,q,zeros,zeros,g)
 
        u = g + Kp.dot(qdes-q) - Kd.dot(dq)
    
        fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+
                 str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
    
        u[0:3] = np.clip(u[0:3],-150,150)
        u[3:6] = np.clip(u[3:6],-28,28)
        
        goal.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=dq, time_from_start=rospy.Duration(0.05))]
        robot_client.send_goal(goal)
        robot_client.wait_for_result()
        
        # Simulacion del robot
        robot.send_command(u)

        t = t+dt
    
        if np.linalg.norm(x-xdes)<0.001:
            break
    
        # Esperar hasta la siguiente  iteracion
        rate.sleep()

    robot_client.cancel_goal()
    fqact.close()
    fqdes.close()
    fxact.close()
    fxdes.close()
    fu.close()
