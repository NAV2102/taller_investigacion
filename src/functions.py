import numpy as np
from copy import copy
from scipy.signal import find_peaks
pi = np.pi
import rbdl




class Robot(object):
    def __init__(self, q0, dq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/ur5_robot2.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.zeros(6)
        rbdl.ForwardDynamics(self.robot, self.q, self.dq, tau, ddq)
        #ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq
        
"""        
class RobotX(object):
    def __init__(self, x0, dx0, ndof, dt):
        self.x = x0    # numpy array (ndof x 1)
        self.dx = dx0  # numpy array (ndof x 1)
        self.H = np.zeros([ndof, ndof])
        self.C = np.zeros([ndof, ndof])
        self.G = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('../urdf/ur5_robot2.urdf')

    def send_command(self, tau):
        zeros = np.zeros(ndof)
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.H)
        rbdl.InverseDynamics(self.robot,self.q,zeros,zeros,self.G)
        rbdl.InverseDynamics(self.robot,self.q,self.dq,zeros,self.C)
        self.C = self.C - self.G
        Hp = J_T.dot(H).dot(JI)
        Cp = J_T.dot(C).dot(JI) - Hp.dot(dJ).dot(JI)
        Gp = J_T.dot(G)
        ddx = np.linalg.inv(Hp).dot(J_T.dot(tau)-Cp-Gp)
        ddq = np.zeros(6)
        rbdl.ForwardDynamics(self.robot, self.q, self.dq, tau, ddq)
        #ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        self.dq = self.dq + self.dt*ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq
"""        

class Capsule(object):
    def __init__(self, p, u, r):
        self.p = np.transpose(np.array(p))
        self.u = np.transpose(np.array(u))
        self.r = r
        # Variables necesarias para calcular la distancia minima
        S1 = u - p
        r11seq = np.sum(np.square(S1))
        r11 = np.power(r11seq,0.5)
        q1 = S1/r11
        self.S1 = S1
        self.r11seq = r11seq
        self.r11 = r11
        self.q1 = q1
        
    def send_positions(self, p, u):
        self.p = np.transpose(np.array(p))
        self.u = np.transpose(np.array(u))
        S1 = u - p
        r11seq = np.sum(np.square(S1))
        r11 = np.power(r11seq,0.5)
        q1 = S1/r11
        self.S1 = S1
        self.r11seq = r11seq
        self.r11 = r11
        self.q1 = q1
        
    def read_positions(self):
        return self.p, self.u
        
    def read_variables(self):
        return self.p, self.u, self.r, self.S1, self.r11seq, self.r11, self.q1


def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea asociada a los parametros DH.
    Retorna una matriz 4x4
    """
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa  = np.sin(alpha)
    ca  = np.cos(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                   [sth,  ca*cth, -sa*cth, a*sth],
                   [0.0,      sa,      ca,     d],
                   [0.0,     0.0,     0.0,   1.0]])
    return T


def ur5_fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    l1 = 1.2892
    #l1 = 0.0892
    l2 = 0.425
    l3 = 0.392
    l4 = 0.09475
    l5 = 0.1093
    l6 = 0.0825
    # Matrices DH
    T1 = dh( l1,        q[0],  0, pi/2)
    T2 = dh(  0, q[1]+2*pi/2, l2,    0)
    T3 = dh(  0,        q[2], l3,    0)
    T4 = dh( l5, q[3]+2*pi/2,  0, pi/2)
    T5 = dh( l4,     q[4]+pi,  0, pi/2)
    T6 = dh( l6,        q[5],  0,    0)
    # Efector final con respecto a la base
    T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T
    

def positions_robot(q):
    """
    Calcular la posicipin de cada una de cada articulacion del
    robot para posicionar las capsulas
    """
    if len(q) < 6:
        q = np.array([0,0,0,0,0,0])
    # Crear una matriz 3x6
    pos = np.zeros((3,6))
    l1 = 1.2892
    #l1 = 0.0892
    l2 = 0.425
    l3 = 0.392
    l4 = 0.09475
    l5 = 0.1093
    l6 = 0.0825
    # Matrices DH
    T1 = dh( l1,        q[0],  0, pi/2)
    T2 = dh(  0, q[1]+2*pi/2, l2,    0)
    T3 = dh(  0,        q[2], l3,    0)
    T4 = dh( l5, q[3]+2*pi/2,  0, pi/2)
    T5 = dh( l4,     q[4]+pi,  0, pi/2)
    T6 = dh( l6,        q[5],  0,    0)
    # Asignar posiciones
    pos[0:3,0] = T1[0:3,3]
    pos[0:3,1] = (T1.dot(T2))[0:3,3]
    pos[0:3,2] = (T1.dot(T2).dot(T3))[0:3,3]
    pos[0:3,3] = (T1.dot(T2).dot(T3).dot(T4))[0:3,3]
    pos[0:3,4] = (T1.dot(T2).dot(T3).dot(T4).dot(T5))[0:3,3]
    pos[0:3,5] = (T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6))[0:3,3]
    return pos


def jacobian(q,n):
    if len(q) < 6:
        q = np.array([0,0,0,0,0,0])
    # Crear una matriz 3x6
    pos = np.zeros((3,6))
    l1 = 1.2892
    #l1 = 0.0892
    l2 = 0.425
    l3 = 0.392
    l4 = 0.09475
    l5 = 0.1093
    l6 = 0.0825
    # Matrices DH
    T1 = dh( l1,        q[0],  0, pi/2)
    T2 = dh(  0, q[1]+2*pi/2, l2,    0)
    T3 = dh(  0,        q[2], l3,    0)
    T4 = dh( l5, q[3]+2*pi/2,  0, pi/2)
    T5 = dh( l4,     q[4]+pi,  0, pi/2)
    T6 = dh( l6,        q[5],  0,    0)
    # Cinematica directa
    T01 = T1
    T02 = T01.dot(T2)
    T03 = T02.dot(T3)
    T04 = T03.dot(T4)
    T05 = T04.dot(T5)
    T06 = T05.dot(T6)
    # Vectores z
    z0 = np.array([0,0,1])
    z1 = T01[0:3,2]
    z2 = T02[0:3,2]
    z3 = T03[0:3,2]
    z4 = T04[0:3,2]
    z5 = T05[0:3,2]
    Z = np.array([z0,z1,z2,z3,z4,z5])
    # Vectores p
    p00 = np.array([0,0,0])
    p01 = T01[0:3,3]
    p02 = T02[0:3,3]
    p03 = T03[0:3,3]
    p04 = T04[0:3,3]
    p05 = T05[0:3,3]
    P = np.array([p00,p01,p02,p03,p04,p05])
    Pn = P[n-1,:]
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    for i in range(n):
        pin = Pn - P[i,:]
        J[0:3,i] = np.cross(Z[i,:],pin)
    return J

    
def jacobian_ur5(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuración articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3,6))
    # Transformación homogénea inicial (usando q)
    Ti = ur5_fkine(q)
    
    # Iteración para la derivada de cada columna
    for i in range(6):
        # Copiar la configuración articular inicial
        dq = copy(q)
        # Incrementar la articulación i-ésima usando un delta
        dq[i] = dq[i] + delta
        # Transformación homogénea luego del incremento (q+delta)
        Tf = ur5_fkine(dq)
        # Aproximación del Jacobiano de posición usando diferencias finitas
        J[0,i] = (Tf[0,3]-Ti[0,3])/delta
        J[1,i] = (Tf[1,3]-Ti[1,3])/delta
        J[2,i] = (Tf[2,3]-Ti[2,3])/delta
        
    return J


def jacobian_pose(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion y orientacion (usando un
    cuaternion). Retorna una matriz de 7x6 y toma como entrada el vector de
    configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    J = np.zeros((7,6))
    # Transformacion homogenea inicial (usando q)
    Ti = ur5_fkine(q)
    Qi = rot2quat(Ti[0:3,0:3])
    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        Tf = ur5_fkine(dq)
        Qf = rot2quat(Tf[0:3,0:3])
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0,i] = (Tf[0,3]-Ti[0,3])/delta
        J[1,i] = (Tf[1,3]-Ti[1,3])/delta
        J[2,i] = (Tf[2,3]-Ti[2,3])/delta
        J[3,i] = (Qf[0]-Qi[0])/delta
        J[4,i] = (Qf[1]-Qi[1])/delta
        J[5,i] = (Qf[2]-Qi[2])/delta
        J[6,i] = (Qf[3]-Qi[3])/delta

    return J



def rot2quat(R):
    """
    Convertir una matriz de rotacion en un cuaternion

    Entrada:
      R -- Matriz de rotacion
    Salida:
      Q -- Cuaternion [ew, ex, ey, ez]

    """
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    #quat[1] = 0.4*(R[2,1]-R[1,2])/quat[0]
    #quat[2] = 0.4*(R[0,2]-R[2,0])/quat[0]
    #quat[3] = 0.4*(R[1,0]-R[0,1])/quat[0]
    
    if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
        quat[1] = 0.0
    else:
        quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
    if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
        quat[2] = 0.0
    else:
        quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
    if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
        quat[3] = 0.0
    else:
        quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)
    
    return np.array(quat)


def quaternionMult(q1, q2):
    quat = 4*[0.,]
    quat[0] = -q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]+q1[0]*q2[0]
    quat[1] =  q1[0]*q2[1]-q1[3]*q2[2]+q1[2]*q2[3]+q1[1]*q2[0]
    quat[2] =  q1[3]*q2[1]+q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]
    quat[3] = -q1[2]*q2[1]+q1[1]*q2[2]+q1[0]*q2[3]+q1[3]*q2[0]
    return np.array(quat)


def TF2xyzquat(T):
    """
    Convert a homogeneous transformation matrix into the a vector containing the
    pose of the robot.

    Input:
      T -- A homogeneous transformation
    Output:
      X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
           is Cartesian coordinates and the last part is a quaternion
    """
    quat = rot2quat(T[0:3,0:3])
    res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)


def skew(w):
    R = np.zeros([3,3])
    R[0,1] = -w[2]; R[0,2] = w[1]
    R[1,0] = w[2];  R[1,2] = -w[0]
    R[2,0] = -w[1]; R[2,1] = w[0]
    return R

 
def ikine_ur5(xdes, q0):
    """
    Calcular la cinemática inversa de UR5 numéricamente a partir de la configuración articular inicial de q0. 
    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001
 
    q  = copy(q0)
    for i in range(max_iter): 
        # Main loop
        #J = jacobian_ur5(q,delta)
        J = jacobian(q,6)
        T = ur5_fkine(q)
        f = T[0:3,3]
        e = xdes - f
        q = q + np.dot(J.T,e)
        q = q_lim(q)
        if (np.linalg.norm(e) < epsilon):
            break
    return q

    
def lidar_filter(med, med_0, alpha = 0.1):
    """
    Filtra los datos obtenidos por el Lidar
    """
    #alpha = 0.1
    
    med_f = alpha*np.array(med) + (1-alpha)*np.array(med_0)
    med_f = med_f.tolist()
    return med_f
    
    
def person_pos_xy(med, ang):
    """
    Calcula la posición de la persona con respecto al robot
    """
    # Distanca x e y con respecto al robot
    dist_x = 0.0
    dist_y = 0.5
    x_person = np.array([10, 10])
    med_p = np.array([10,10])
    ang_p = np.array([0, 0])
    
    med = np.array(med)
    ang = np.array(ang)
    
    pos,_ = find_peaks(-med,distance=10)
    if len(pos) >= 2:
        med_p = med[pos]
        ang_p = ang[pos]
    
    r1 = med_p[0]+0.05
    th1 = ang_p[0]
    r2 = med_p[1]+0.05
    th2 = ang_p[1]
    
    x1 = np.array([r1*np.cos(th1)+dist_x, r1*np.sin(th1)+dist_y])
    x2 = np.array([r2*np.cos(th2)+dist_x, r2*np.sin(th2)+dist_y])
    
    x_person = (x1+x2)/2
    x_person = np.array([-x_person[0], x_person[1], 0.3, -x_person[0], x_person[1], 2.0])
    x_person = x_person.tolist()
    
    return x_person
    
    
def sat(x):
    sat_x = x
    for i in range(len(x)):
        if np.absolute(x[i]) <= 1:
            sat_x[i] = x[i]
        else:
            sat_x[i] = np.sign(x[i])    
    return sat_x    
    
    
def ley_adap(M,P,dB,alpha):
    
    dM = M
    for i in range(7):
        if M[i,i] >= P[i,i]:
            dM[i,i] = -alpha*dB[i]
        else:
            dM[i,i] = P[i,i]
    return dM
    
    
def q_lim(q):
    q[0] = np.clip(q[0],-2*pi,2*pi)
    q[1] = np.clip(q[1],-pi/2,pi/2)
    q[2] = np.clip(q[2],-pi/2,pi/2)
    q[3] = np.clip(q[3],-2*pi,2*pi)
    q[4] = np.clip(q[4],-2*pi,2*pi)
    q[5] = np.clip(q[5],-2*pi,2*pi)
    return q

   
    
def QR_min_dist(capsule1, capsule2):
    """
    Calcular la distancia minima entre capsulas con
    la factorizacion QR
    """
    p1, u1, _, S1, _, r11, q1    = capsule1.read_variables()
    p2, u2, _, S2, r11seq, _, _ = capsule2.read_variables()
    q2 = S2 - (S2.dot(q1))*q1
    q2 = q2/np.linalg.norm(q2)
    r12 = q1.dot(S2)
    y = p1-p2
    
    n1 = np.zeros(2)
    n2 = np.zeros(2)
    n3 = np.zeros(2)
    n4 = np.zeros(2)
    
    n4[0] = q1.dot(y)
    n24r22 = n4[0]*r12 - S1.dot(y)
    
    n3[0] = n4[0] + r11
    n14seq = n4[0]*n4[0]
    yTy = y.dot(y)
    r22seq = r11seq - r12*r12
    n1[0] = n4[0] - r12
    n2[0] = n1[0] + r11
    
    r22 = np.power(r22seq,0.5)
    Q = np.array([q1,q2]).T
    R = np.array([[r11,r12],[0,r22]])
    x = np.array([[0,1,1,0],[1,1,0,0]])
    N = R.dot(x) + np.transpose(Q).dot(np.array([y]).T)
    
    n4[1] = N[1][3] 
    n3[1] = n4[1]
    n2[1] = N[1][1]
    n1[1] = N[1][0]

    # Verificar si el punto esta en la horizontal inferior
    if (n24r22 > 0 and n4[0]*n3[0] < 0):
        #print("Punto en el segmento horizontal inferior")
        umin = np.array([0,n4[1]])
        dmin2 = yTy - n14seq
        return umin, dmin2
    
    # Verificar si los cilindros son paralelos
    if (r22seq == 0):
        if n4[0]*n2[0] <= 0:
            #print("Paralelos entre 2 y 4")
            umin = np.array([0,n4[1]])
            dmin2 = yTy - n14seq
            return umin, dmin2
        if n1[0]*n3[0] <= 0:
            #print("Paralelos entre 1 y 3")
            umin = np.array([0,n4[1]])
            dmin2 = yTy - n14seq
            return umin, dmin2
        if r12 < 0:
            if n4[0] > 0:
                #print("Paralelo punto n4")
                umin = np.array([n4[0],n4[1]])
                dmin2 = yTy
                return umin, dmin2
            elif n2[0] < 0:
                #print("Paralelo punto n2")
                umin = np.array([n2[0],n2[1]])
                dmin2 = n2[0]*n2[0] + yTy - n14seq
                return umin, dmin2
        else:
            if n1[0] > 0:
                #print("Paralelo punto n1")
                umin = np.array([n1[0],n1[1]]) 
                dmin2 = n1[0]*n1[0] + yTy - n14seq
                return umin, dmin2
            elif n3[0] < 0:
                #print("Paralelo punto n3")
                umin = np.array([n3[0],n3[1]]) 
                dmin2 = n3[0]*n3[0] + yTy - n14seq
                return umin, dmin2
    
    n21r22 = r22seq + n24r22
    
    # Verificar si el punto esta en la horizontal superior
    if (n21r22 < 0 and n1[0]*n2[0] < 0):
        umin = np.array([0, n1[1]])  
        dmin2 = (n21r22*n21r22 - n24r22*n24r22)/r22seq + yTy - n14seq
        return umin, dmin2
        
    m4 = n1[0] + r12*n21r22/r22seq
    m3 = m4 + r11
    
    # Verificar si el punto esta dentro del paralelogramo
    if (n21r22*n24r22 < 0 and m3*m4 < 0):
        #print("Punto dentro del paralelogramo")
        umin = np.array([0, 0]) 
        dmin2 = yTy - n14seq - n24r22*n24r22/r22seq
        return umin, dmin2
    # Verificar si el punto mas cerca al origen esta en los
    # segmentos oblicuos
    elif (m4*m4 < m3*m3):
        # Cerca al oblicuo izquierdo
        #print("Oblicuo izquierdo")
        u = ((-n4[0])*(n1[0]-n4[0])+(-n4[1])*(n1[1]-n4[1]))/(np.power(n1[0]-n4[0],2)+np.power(n1[1]-n4[1],2))
        if u >= 1:
            umin = np.array([n1[0], n1[1]])
        elif u <= 0:
            umin = np.array([n4[0], n4[0]])
        else:
            umin = np.array([n4[0]+u*(n1[0]-n4[0]),n4[1]+u*(n1[1]-n4[1])])        
        
        if r12*n1[0] > n21r22:
            n11seq = n1[0]*n1[0]
            n12seq = n2[0]*n2[0]
            if n11seq < n12seq:
                #umin = np.array([n1[0], n1[1]])
                dmin2 = n11seq + yTy - n14seq + (n21r22*n21r22 - n24r22*n24r22)/r22seq
                return umin, dmin2
            else:
                #umin = np.array([n2[0], n1[1]])
                dmin2 = n12seq + yTy - n14seq + (n21r22*n21r22 - n24r22*n24r22)/r22seq
                return umin, dmin2
        elif r12*n4[0] < n24r22:
            if n3[0]*n3[0] > n14seq:
                #umin = np.array([n4[0],n4[1]]) 
                dmin2 = yTy
                return umin, dmin2
            else:
                #umin = np.array([n3[0],n4[1]]) 
                dmin2 = n3[0]*n3[0] + yTy - n14seq
                return umin, dmin2
        else:
            dmin2 = m4*m4*r22seq/r11seq + yTy - n14seq - n24r22*n24r22/r22seq
            return umin, dmin2
    else:
        # Cerca del oblicuo derecho
        #print("Oblicuo derecho")
        u = ((-n3[0])*(n2[0]-n3[0])+(-n3[1])*(n2[1]-n3[1]))/(np.power(n2[0]-n3[0],2)+np.power(n2[1]-n3[1],2))
        if u >= 1:
            umin = np.array([n2[0], n2[1]])
        elif u <= 0:
            umin = np.array([n3[0], n3[0]])
        else:
            umin = np.array([n3[0]+u*(n2[0]-n3[0]),n3[1]+u*(n2[1]-n3[1])])
            
        if (n2[0]*r12 > n21r22):
            n11seq = n1[0]*n1[0]
            n12seq = n2[0]*n2[0]
            if n11seq < n12seq:
                #umin = np.array([n1[0], n1[1]])
                dmin2 = n11seq + yTy - n14seq + (n21r22*n21r22 - n24r22*n24r22)/r22seq
                return umin, dmin2
            else: 
                #umin = np.array([n2[0], n1[1]])
                dmin2 = n12seq + yTy - n14seq + (n21r22*n21r22 - n24r22*n24r22)/r22seq
                return umin, dmin2
        elif (n3[0]*r12 < n24r22):
            if n3[0]*n3[0] < n14seq:
                #umin = np.array([n3[0],n3[1]]) 
                dmin2 = n3[0]*n3[0] + yTy - n14seq
                return umin, dmin2
            else:
                #umin = np.array([n4[0],n4[1]])
                dmin2 = yTy
                return umin, dmin2
        else:
            dmin2 = m3*m3*r22seq/r11seq + yTy - n14seq - n24r22*n24r22/r22seq
            return umin, dmin2
        
    
def segment_capsule(capsule1, capsule2, umin):
    p1, u1, ra1, S1, _, r11, q1    = capsule1.read_variables()
    p2, u2, ra2, S2, r11seq, _, _ = capsule2.read_variables()
    q2 = S2 - (S2.dot(q1))*q1
    q2 = q2/np.linalg.norm(q2)
    r12 = q1.dot(S2)
    r22seq = r11seq - r12*r12
    r22 = np.power(r22seq,0.5)
    y = p1-p2
    
    Q = np.array([q1,q2]).T
    R = np.array([[r11,r12],[0,r22]])
    A = Q.dot(R)
    
    xmin = np.linalg.inv(R).dot(umin-np.transpose(Q).dot(y))
    #xmin = np.clip(xmin,0,1)
    v1 = p1 + A.dot(np.array([[-1,0],[0,0]])).dot(xmin)
    v2 = p2 + A.dot(np.array([[0,0],[0,1]])).dot(xmin)
    v1[2] = v2[2] 
    return v1, v2, xmin
    
    
