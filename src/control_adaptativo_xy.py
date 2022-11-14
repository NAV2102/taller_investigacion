#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages

import rbdl


rospy.init_node("control_adaptativo")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = BallMarker(color['RED'])
bmarker_deseado = BallMarker(color['BLUE'])
# Archivos donde se almacenara los datos
fM = open("/home/nicolas/catkin_ws/src/proyecto/texto/M.dat", "w")
fdB = open("/home/nicolas/catkin_ws/src/proyecto/texto/dB.dat", "w")
fu = open("/home/nicolas/catkin_ws/src/proyecto/texto/u.dat", "w")
fq = open("/home/nicolas/catkin_ws/src/proyecto/texto/q.dat", "w")
fx = open("/home/nicolas/catkin_ws/src/proyecto/texto/x.dat", "w")

# Nombres de las articulaciones
jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Valores del mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames

# =============================================================
# Configuración articular deseada
q_des = np.array([1.0, -1.0, 1.0, 1.3, -1.5, 1.0])
x_des = np.zeros(7)
# Configuracion inicial
q_0  = np.array([0, -0.5, 0.8, -2.2, -1.6, 0.0])
dq_0 = np.zeros(6)
ddq_0 = np.zeros(6)
T = ur5_fkine(q_0)
Q = rot2quat(T)
x = np.zeros(7)
x[0:3] = T[0:3,3]
x[3:7] = Q
# =============================================================

# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q_0
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('/home/nicolas/catkin_ws/src/proyecto/urdf/ur5_robot.urdf')
ndof   = modelo.q_size     # Grados de libertad

# Frecuencia del envio (en Hz)
freq = 100
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q_0, dq_0, ddq_0, ndof, dt)

# Initial joint configuration
q  = robot.read_joint_positions()
dq = robot.read_joint_velocities()
# Se definen las ganancias del controlador

lamb  = 1000*np.diag([1.0, 1.5, 4.0, 1.0/10, 1.0/10, 1.0/10, 1.0/10])
M = 1*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])       # Matriz de adaptacion
"""
Kp = np.power(lamb,2)
Kd = 2*lamb
"""
Kp = lamb
Kd = 2*np.sqrt(Kp)
alpha = np.array([0.1, 0.1, 0.1, 0.0001, 0.01, 0.01, 0.01])

t = 0.0
u = np.zeros(ndof)          # Reemplazar por la ley de control
P = np.copy(M)
e = np.zeros(7)
rospy.sleep(2)

while not rospy.is_shutdown():
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()
    
    mov = pi/2*np.cos(pi*t/8) - pi/2
    mov2 = pi/8*np.cos(3*pi*t/4) - pi/4  
    mov3 = -pi/8*np.cos(3*pi*t/4) + pi/4
    dmov = -np.sin(pi*t/8)*pi*pi/16 
    dmov2 = -np.sin(3*pi*t/4)*pi*pi*3/32
    dmov3 = np.sin(3*pi*t/4)*pi*pi*3/32
    ddmov = -np.cos(pi*t/8)*pi*pi*pi/128
    ddmov2 = -np.cos(3*pi*t/4)*pi*pi*pi*9/128
    ddmov3 = np.cos(3*pi*t/4)*pi*pi*pi*9/128   
    
    # Posicion deseada     
    q_des = np.array([mov, mov2, mov3, -2.2, -1.6, 0.0])
    dq_des = np.array([dmov, dmov2, dmov3, 0.0, 0.0, 0.0])
    ddq_des = np.array([ddmov, ddmov2, ddmov3, 0.0, 0.0, 0.0])
    
    Td = ur5_fkine(q_des)
    Qd = rot2quat(Td)
    x_des[0:3] = Td[0:3,3]
    x_des[3:7] = Qd
    
    # Posicion actual
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    
    T = ur5_fkine(q)
    Q = rot2quat(T)
    x[0:3] = T[0:3,3]
    x[3:7] = Q
    
    # Error
    Qe = quatError(Qd,Q)
    #Qe = quatMult(Qd,Q)
    e[0:3] = x_des[0:3] - x[0:3]
    e[3:7] = Qe
    
    # Derivada de la posicion
    J = jacobian_pose(q)
    JT = np.transpose(J)
    #JI = JT.dot(np.linalg.inv(J.dot(JT)))
    dx = J.dot(dq)
    de = -dx
    print(t, Qe[0])
    # Superficie de control
    s = de + 0.5*Kd.dot(e)
    ds = -0.5*Kd.dot(de) - Kp.dot(e)
    
    # Funcion de costo
    bi = 0.5*np.power(s,2)
    dB = np.multiply(s,ds)

    rbdl.CompositeRigidBodyAlgorithm(modelo,q,M)
    # Ley de adaptacion
    for i in range(len(x_des)):
        dM = -alpha[i]*dB[i]
        M[i,i] = M[i,i] + dt*dM
        if M[i,i] <= P[i,i]:
            M[i,i] = P[i,i]
    
    # Torque
    u = JT.dot(M.dot(Kd.dot(de) + Kp.dot(e)))
    u[0:3] = np.clip(u[0:3],-150,150)
    u[3:6] = np.clip(u[3:6],-28,28)
    
    # Simulacion del robot 
    robot.send_command(u)
    
    # Almacenamiento de datos
    fM.write(str(t)+' '+str(M[0,0])+' '+str(M[1,1])+' '+str(M[2,2])+' '+str(M[3,3])+' '+str(M[4,4])+' '+str(M[5,5])+' '+str(M[6,6])+'\n')
    fdB.write(str(t)+' '+str(dB[0])+' '+str(dB[1])+' '+str(dB[2])+' '+str(dB[3])+' '+str(dB[4])+' '+str(dB[5])+' '+str(dB[6])+'\n')
    fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
    fq.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+str(q[2])+' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q_des[0])+' '+str(q_des[1])+' '+str(q_des[2])+' '+str(q_des[3])+' '+str(q_des[4])+' '+str(q_des[5])+'\n')
    fx.write(str(t)+' '+str(x[0])+' '+str(x[1])+' '+str(x[2])+' '+str(x_des[0])+' '+str(x_des[1])+' '+str(x_des[2])+'\n')

    # Publicacion del mensaje
    
    jstate.position = q
    pub.publish(jstate)
    
    bmarker_deseado.xyz(x_des[0:3])
    bmarker_actual.xyz(x[0:3])
    t = t+dt
    
    if np.linalg.norm(e)<0.001 or t>=8:
        break
    
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fM.close()
fdB.close()
fu.close()
fq.close()
fx.close()
