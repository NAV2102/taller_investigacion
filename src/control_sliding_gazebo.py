#!/usr/bin/env python

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

    rospy.init_node("control_sliding")

    robot_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    # Archivos donde se almacenara los datos
    fqact = open("/home/nicolas/catkin_ws/src/proyecto/texto/qactual.dat", "w")
    fqdes = open("/home/nicolas/catkin_ws/src/proyecto/texto/qdeseado.dat", "w")

    # Nombres de las articulaciones
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # =============================================================
    # Configuracion articular inicial (en radianes)
    q = np.array([0.0, -1.2, 1.7, -2.2, -1.6, 0.0])
    # Velocidad inicial
    dq = np.array([0., 0., 0., 0., 0., 0.])
    # Configuracion articular deseada
    #q_des = np.array([-pi/2, -0.5, 0.5, -pi/2, -pi/2, 0.0])
    q_des = np.array([-1.0, -1.0, 1.0, 1.3, -1.5, 1.0])
    dq_des = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ddq_des = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # =============================================================
    # Posicion resultante de la configuracion articular deseada
    xdes = ur5_fkine(q_des)[0:3,3]
    
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = jnames
    
    # Initial position
    Q0 = [0, -0.5, 0.8, -2.2, -1.6, 0.0]
    goal.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=dq,time_from_start=rospy.Duration(2.0))]
    robot_client.send_goal(goal)
    robot_client.wait_for_result()
    rospy.sleep(1)
    
    # Modelo RBDL
    modelo = rbdl.loadModel('/home/nicolas/catkin_ws/src/proyecto/urdf/ur5_robot.urdf')
    ndof   = modelo.q_size     # Grados de libertad
    
    # Frecuencia del envio (en Hz)
    freq = 50
    dt = 1.0/freq
    rate = rospy.Rate(freq)
    
    # Simulador dinamico del robot
    robot = Robot(np.array(Q0), dq, ndof, dt)
    
    # Se definen las ganancias del controlador
    n     = 0.1*np.array([200.0, 50.0, 10.0, 50.0, 50.0, 1.0])
    lamb  = 20*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    alpha = 0.2
    
    # Bucle de ejecucion continua
    t = 0.0
    u     = np.zeros(ndof)          # Reemplazar por la ley de control
    b     = np.zeros(ndof)          # Para el vector de efectos no lineales
    H     = np.zeros([ndof, ndof])  # Para la matriz de inercia
    H0    = np.zeros([ndof, ndof])
    
    q  = robot.read_joint_positions()
    rbdl.CompositeRigidBodyAlgorithm(modelo,q,H0)
    
    while not rospy.is_shutdown():
        robot_client.cancel_goal()
        
        mov = pi/2*np.cos(pi*t/8) - pi/2
        mov2 = pi/8*np.cos(3*pi*t/4) - pi/4  
        dmov = -np.sin(pi*t/8)*pi*pi/16 
        dmov2 = -np.sin(3*pi*t/4)*pi*pi*3/32 
        ddmov = -np.cos(pi*t/8)*pi*pi*pi/128
        ddmov2 = -np.cos(3*pi*t/4)*pi*pi*pi*9/128    
        q_des = np.array([mov, mov2, 0.8, -2.2, -1.6, 0.0])
        dq_des = np.array([dmov, dmov2, 0.0, 0.0, 0.0, 0.0])
        ddq_des = np.array([ddmov, ddmov2, 0.0, 0.0, 0.0, 0.0])
        
        # Leer valores del simulador
        q  = robot.read_joint_positions()
        dq = robot.read_joint_velocities()
        # Posicion actual del efector final
        x = ur5_fkine(q)[0:3,3]
        
        # Calcular el error
        e  =  q -  q_des
        de = dq - dq_des
        
        # Superficie de control
        s = de + lamb.dot(e)
        
        # Almacenamiento de datos
        fqact.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+ str(q[2])+
                ' '+ str(q[3])+' '+str(q[4])+' '+str(q[5])+'\n')
        fqdes.write(str(t)+' '+str(q_des[0])+' '+str(q_des[1])+' '+ str(q_des[2])+' '+ str(q_des[3])+' '+str(q_des[4])+' '+str(q_des[5])+'\n')
        
        # ----------------------------
        # Control dinamico (COMPLETAR)
        # ----------------------------
        # Calcular H y b
        rbdl.CompositeRigidBodyAlgorithm(modelo,q,H)
        rbdl.NonlinearEffects(modelo,q,dq,b)
        
        dH = (H-H0)/dt
        H0 = H
        
        k = n + np.absolute(alpha*H.dot(ddq_des-lamb.dot(de)) + alpha*b + (1-alpha)*dH.dot(dq_des-lamb.dot(e)))
        
        u = b + H.dot(ddq_des) - H.dot(lamb).dot(de) - np.multiply(k,sat(s))
        
        u[0:3] = np.clip(u[0:3],-150,150)
        u[3:6] = np.clip(u[3:6],-28,28)

        #fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+
        #         str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
        
        # Simulacion del robot
        robot.send_command(u)
        
        #q=q_lim(q)
        goal.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=dq, time_from_start=rospy.Duration(dt))]
        robot_client.send_goal(goal)
        robot_client.wait_for_result()
        
        t = t+dt
    
        if t >= 16:
            break
    
        # Esperar hasta la siguiente  iteracion
        rate.sleep()

    robot_client.cancel_goal()

    fqact.close()
    fqdes.close()


