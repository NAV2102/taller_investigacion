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
dq_des = np.zeros(6)
ddq_des = np.zeros(6)
# Obtener el vector de posicion y orientacion
x_des = ur5_fkine(q_des)[0:3,3]
# Configuracion inicial
q_0  = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dq_0 = np.zeros(6)
ddq_0 = np.zeros(6)

# Posicion inicial
x_0 = ur5_fkine(q_0)
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
ddq = robot.read_joint_accelerations()

# Se definen las ganancias del controlador

lamb  = 500*np.diag([1.0, 2.0, 2.0, 2.0, 1.0, 1.0])
M = 0.0*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])       # Matriz de adaptacion
Kp = lamb
Kd = 4*np.sqrt(Kp)
alpha = np.array([0.001, 0.001, 0.0001, 0.0001, 0.0001, 0.0001])
"""
Kp = np.power(lamb,2)
Kd = 2*lamb
"""
t = 0.0
u = np.zeros(ndof)          # Reemplazar por la ley de control
P = np.copy(M)
#dM = np.zeros((ndof,ndof))
rospy.sleep(2)
#print(Kd)
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
    
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    ddq = robot.read_joint_accelerations()
    
    # Calcular el error
    e = q_des - q
    de = dq_des - dq
    
    # Superficie de control
    s = de + 0.7*Kd.dot(e)
    ds = -0.2*Kd.dot(de) - Kp.dot(e)
    
    # Funcion de costo
    bi = 0.5*np.power(s,2)
    # Derivada de la funcion de costo
    dB = np.multiply(s,ds)
    
    rbdl.CompositeRigidBodyAlgorithm(modelo,q,M)
    # Ley de adaptacion
    for i in range(len(q_des)):
        dM = -alpha[i]*dB[i]
        M[i,i] = M[i,i] + dt*dM
        if M[i,i] <= P[i,i]:
            M[i,i] = P[i,i]
    
    # Torque
    u = M.dot(Kd.dot(de) + Kp.dot(e))
    u[0:3] = np.clip(u[0:3],-150,150)
    u[3:6] = np.clip(u[3:6],-28,28)
    #u[5] = np.clip(u[5],-5,5)
    
    # Almacenamiento de datos
    fM.write(str(t)+' '+str(M[0,0])+' '+str(M[1,1])+' '+str(M[2,2])+' '+str(M[3,3])+' '+str(M[4,4])+' '+str(M[5,5])+'\n')
    fdB.write(str(t)+' '+str(dB[0])+' '+str(dB[1])+' '+str(dB[2])+' '+str(dB[3])+' '+str(dB[4])+' '+str(dB[5])+'\n')
    fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
    fq.write(str(t)+' '+str(q[0])+' '+str(q[1])+' '+str(q[2])+' '+str(q[3])+' '+str(q[4])+' '+str(q[5])+' '+str(q_des[0])+' '+str(q_des[1])+' '+str(q_des[2])+' '+str(q_des[3])+' '+str(q_des[4])+' '+str(q_des[5])+'\n')
    
    
    
    # Simulacion del robot 
    robot.send_command(u)
    
    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    
    x = ur5_fkine(q)[0:3,3]
    x_des = ur5_fkine(q_des)[0:3,3]
    bmarker_deseado.xyz(x_des)
    bmarker_actual.xyz(x)
    t = t+dt
    
    if np.linalg.norm(e)<0.001 or t>=8:
        break
    
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fM.close()
fdB.close()
fu.close()
fq.close()
