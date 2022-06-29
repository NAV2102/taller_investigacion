#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from roslib import packages

import rbdl


rospy.init_node("control_adaptativo")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker_actual  = FrameMarker()
bmarker_deseado = FrameMarker()
# Archivos donde se almacenara los datos
fM = open("../texto/M.dat", "w")
fdB = open("../texto/dB.dat", "w")
fu    = open("../texto/u.dat", "w")

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
q_des = np.array([0.1, -1.0, 1.7, -2.2, -1.6, 0.0])
# Obtener el vector de posicion y orientacion
T_des = ur5_fkine(q_des)
quat_des = rot2quat(T_des[0:3,0:3])
xdes = TF2xyzquat(T_des)
dxdes = np.zeros(7)
ddxdes = np.zeros(7)
# Configuracion inicial
q0  = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])
dq = np.array([0., 0., 0., 0., 0., 0.])

# Posicion inicial
T = ur5_fkine(q0)
x0 = TF2xyzquat(T)
# =============================================================



# Copiar la configuracion articular en el mensaje a ser publicado
jstate.position = q0
pub.publish(jstate)

# Modelo RBDL
modelo = rbdl.loadModel('../urdf/ur5_robot2.urdf')
ndof   = modelo.q_size     # Grados de libertad

# Frecuencia del envio (en Hz)
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)

# Simulador dinamico del robot
robot = Robot(q0, dq, ndof, dt)

# Initial joint configuration
q  = robot.read_joint_positions()
dq = robot.read_joint_velocities()
T = ur5_fkine(q)
x = TF2xyzquat(T)
quat = x[3:7]
dx0 = np.zeros(7)

# Se definen las ganancias del controlador
lamb  = 40*np.diag([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])

t = 0.0
u = np.zeros(ndof)          # Reemplazar por la ley de control
M = 1*np.identity(7)        # Matriz de adaptacion
P = M

while not rospy.is_shutdown():
    # Tiempo actual (necesario como indicador para ROS)
    jstate.header.stamp = rospy.Time.now()
    
    # Calculo del jacobiano
    J = jacobian_pose(q)
    JT = np.transpose(J)
    JI = JT.dot(np.linalg.inv(J.dot(JT)))
    J_T = np.transpose(JI)
    
    
    
    dx = J.dot(dq)
    ddx = (dx-dx0)/dt
    dx0 = dx
    #print("xdes: ",xdes)
    #print("x: ",x)
    #print("dx: ",dx)
    # Errores 
    ep = xdes - x
    qe = quaternionMult(quat_des, quat)
    eo = np.array([qe[0]-1, qe[1], qe[2], qe[3]])
    # Calcular el error
    e = np.array([ep[0], ep[1], ep[2], eo[0], eo[1], eo[2], eo[3]])
    de = dxdes - dx
    dde = ddxdes - ddx
    
    # Superficie de control
    s = de + lamb.dot(e)
    ds = dde + lamb.dot(de)
    
    # Funcion de costo
    bi = 0.5*np.power(s,2)
    # Derivada de la funcion de costo
    dB = np.multiply(s,ds)
    
    # Torque
    u = JT.dot(M.dot(2*lamb.dot(de) + np.power(lamb,2).dot(e)))
    u[0:3] = np.clip(u[0:3],-150,150)
    u[3:6] = np.clip(u[3:6],-28,28)
    
    # Simulacion del robot 
    robot.send_command(u)
    
    # Almacenamiento de datos
    fM.write(str(t)+' '+str(M[0,0])+' '+str(M[1,1])+' '+str(M[2,2])+' '+str(M[3,3])+' '+str(M[4,4])+' '+str(M[5,5])+' '+str(M[6,6])+'\n')
    fdB.write(str(t)+' '+str(dB[0])+' '+str(dB[1])+' '+str(dB[2])+' '+str(dB[3])+' '+str(dB[4])+' '+str(dB[5])+' '+str(dB[6])+'\n')
    fu.write(str(t)+' '+str(u[0])+' '+str(u[1])+' '+str(u[2])+' '+
             str(u[3])+' '+str(u[4])+' '+str(u[5])+'\n')
    
    # Ley de adaptacion
    dM = ley_adap(M,P,dB,0.0000001)
    # Integral de la ley de adaptacion
    M = M + dt*dM
    # Actualizar la funcion de costo
    b0 = bi
    
    # actualizar parametros
    q  = robot.read_joint_positions()
    dq = robot.read_joint_velocities()
    T = ur5_fkine(q)
    x = TF2xyzquat(T)
    quat = x[3:7]

    # Publicacion del mensaje
    jstate.position = q
    pub.publish(jstate)
    
    bmarker_deseado.setPose(xdes)
    bmarker_actual.setPose(x)
    t = t+dt
    
    if np.linalg.norm(e)<0.001:
        break
    
    # Esperar hasta la siguiente  iteracion
    rate.sleep()

fM.close()
fdB.close()
fu.close()
