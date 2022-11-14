#!/usr/bin/env python

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from functions import *
from markers import *
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
    fM = open("/home/nicolas/catkin_ws/src/proyecto/texto/M.dat", "w")
    fdB = open("/home/nicolas/catkin_ws/src/proyecto/texto/dB.dat", "w")
    fu = open("/home/nicolas/catkin_ws/src/proyecto/texto/u.dat", "w")
    fq = open("/home/nicolas/catkin_ws/src/proyecto/texto/q.dat", "w")
    fx = open("/home/nicolas/catkin_ws/src/proyecto/texto/x.dat", "w")
    
    bmarker_actual  = BallMarker(color['RED'])
    bmarker_deseado = BallMarker(color['BLUE'])
    
    # Nombres de las articulaciones
    jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # =============================================================
    # Configuracion articular inicial (en radianes)
    q = np.array([-pi/2, -0.5, 0.8, -2.2, -1.6, 0.0])
    # Velocidad inicial
    dq = np.array([0., 0., 0., 0., 0., 0.])
    # Configuracion articular deseada
    #q_des = np.array([-pi/2, -0.5, 0.5, -pi/2, -pi/2, 0.0])
    q_des = np.array([-pi/2, -0.5, 0.8, -2.2, -1.6, 0.0])
    dx_des = np.zeros(3)
    ddx_des = np.zeros(3)
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
    freq = 100
    dt = 1.0/freq
    rate = rospy.Rate(freq)
    dx0 = 0
    # Simulador dinamico del robot
    robot = Robot(np.array(Q0), dq,np.array([0.0]*6), ndof, dt)
    
    # Se definen las ganancias del controlador
    lamb  = 80*np.diag([1.0, 1.0, 1.0])
    M = 1*np.diag([1.0, 1.0, 1.0])       # Matriz de adaptacion
    Kp = np.power(lamb,2)
    Kd = 2*lamb
    alpha = 0.00001
    
    # Bucle de ejecucion continua
    t = 0.0
    u = np.zeros(ndof)          # Reemplazar por la ley de control
    P = np.copy(M)
    
    while not rospy.is_shutdown():
        robot_client.cancel_goal()
        
        mov = pi/2*np.cos(pi*t/8) - pi/2
        mov2 = pi/8*np.cos(3*pi*t/4) - pi/4  
        mov3 = -pi/8*np.cos(3*pi*t/4) + pi/4
        dmov = -np.sin(pi*t/8)*pi*pi/16 
        dmov2 = -np.sin(3*pi*t/4)*pi*pi*3/32
        dmov3 = np.sin(3*pi*t/4)*pi*pi*3/32
        ddmov = -np.cos(pi*t/8)*pi*pi*pi/128
        ddmov2 = -np.cos(3*pi*t/4)*pi*pi*pi*9/128
        ddmov3 = np.cos(3*pi*t/4)*pi*pi*pi*9/128   
         
        q_des = np.array([mov, mov2, mov3, -2.2, -1.6, 0.0])
        dq_des = np.array([dmov, dmov2, dmov3, 0.0, 0.0, 0.0])
        ddq_des = np.array([ddmov, ddmov2, ddmov3, 0.0, 0.0, 0.0])
        
        x_des = ur5_fkine(q_des)[0:3,3]
        # Tiempo actual (necesario como indicador para ROS)
        q  = robot.read_joint_positions()
        #q  = q_lim(q)
        dq = robot.read_joint_velocities()
        ddq = robot.read_joint_accelerations()
        
        J = jacobian_ur5(q)
        JT = np.transpose(J)
        JI = JT.dot(np.linalg.inv(J.dot(JT)))
        J_T = np.transpose(JI)
        
        x = ur5_fkine(q)[0:3,3]
        dx = J.dot(dq)
        ddx = (dx - dx0)/dt
        dx0 = dx

        e = x_des - x
        de = dx_des - dx
        dde = ddx_des - ddx
        
        # Superficie de control
        s = de + lamb.dot(e)
        ds = dde + lamb.dot(de)
    
        # Funcion de costo
        bi = 0.5*np.power(s,2)
        # Derivada de la funcion de costo
        dB = np.multiply(s,ds)
        
        
        for i in range(3):
            if M[i,i] >= P[i,i]:
                dM = -alpha*dB[i]
                M[i,i] = M[i,i] + dt*dM
            else:
                M[i,i] = P[i,i]
        
        u = JT.dot(M.dot(ddx + Kd.dot(de) + Kp.dot(e)))
        u[0:3] = np.clip(u[0:3],-100,100)
        u[3:6] = np.clip(u[3:6],-30,30)

        # Simulacion del robot
        robot.send_command(u)
        
        #q=q_lim(q)
        goal.trajectory.points = [ JointTrajectoryPoint(positions=q, velocities=dq, time_from_start=rospy.Duration(dt))]
        robot_client.send_goal(goal)
        robot_client.wait_for_result()
        
        # Almacenamiento de datos
        fM.write(str(t)+' '+str(M[0,0])+' '+str(M[1,1])+' '+str(M[2,2])+'\n')
        fdB.write(str(t)+' '+str(dB[0])+' '+str(dB[1])+' '+str(dB[2])+'\n')
        fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
        fq.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+str(q[2])+' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q_des[0])+' '+str(q_des[1])+' '+str(q_des[2])+' '+str(q_des[3])+' '+str(q_des[4])+' '+str(q_des[5])+'\n')
        fx.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+' '+str(x_des[0])+' '+str(x_des[1])+' '+str(x_des[2])+'\n')
        
        t = t+dt
        
        bmarker_deseado.xyz(x_des)
        bmarker_actual.xyz(x)
        
        if t >= 8:
            break
    
        # Esperar hasta la siguiente  iteracion
        rate.sleep()

    robot_client.cancel_goal()

    fM.close()
    fdB.close()
    fu.close()
    fq.close()
    fx.close()

