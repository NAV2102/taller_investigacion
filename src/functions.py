import numpy as np
from copy import copy
from scipy.signal import find_peaks
pi = np.pi
import rbdl




class Robot(object):
    def __init__(self, q0, dq0, ddq0, ndof, dt):
        self.q = q0    # numpy array (ndof x 1)
        self.dq = dq0  # numpy array (ndof x 1)
        self.ddq = ddq0
        self.M = np.zeros([ndof, ndof])
        self.b = np.zeros(ndof)
        self.dt = dt
        self.robot = rbdl.loadModel('/home/nicolas/catkin_ws/src/proyecto/urdf/ur5_robot.urdf')

    def send_command(self, tau):
        rbdl.CompositeRigidBodyAlgorithm(self.robot, self.q, self.M)
        rbdl.NonlinearEffects(self.robot, self.q, self.dq, self.b)
        ddq = np.zeros(6)
        rbdl.ForwardDynamics(self.robot, self.q, self.dq, tau, ddq)
        #ddq = np.linalg.inv(self.M).dot(tau-self.b)
        self.q = self.q + self.dt*self.dq
        #self.q = q_lim(self.q)
        self.dq = self.dq + self.dt*ddq
        self.ddq = ddq

    def read_joint_positions(self):
        return self.q

    def read_joint_velocities(self):
        return self.dq
        
    def read_joint_accelerations(self):
        return self.ddq
           

class Capsule(object):
    def __init__(self, p, u, r):
        self.p = np.transpose(np.array(p))
        self.u = np.transpose(np.array(u))
        self.r = r
        
    def send_positions(self, p, u):
        self.p = np.transpose(np.array(p))
        self.u = np.transpose(np.array(u))
        
    def read_positions(self):
        return self.p, self.u
        
    def read_variables(self):
        return self.p, self.u, self.r


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


def transform(q):
    l1 = 0.0892
    l2 = 0.425
    l3 = 0.392
    l4 = 0.09475
    l5 = 0.1093
    #l6 = 0.0825
    l6 = 0.1625
    # Matrices DH
    T1 = dh( l1,        q[0],  0, pi/2)
    T2 = dh(  0, q[1]+2*pi/2, l2,    0)
    T3 = dh(  0,        q[2], l3,    0)
    T4 = dh( l5, q[3]+2*pi/2,  0, pi/2)
    T5 = dh( l4,     q[4]+pi,  0, pi/2)
    T6 = dh( l6,        q[5],  0,    0)
    # Transformadas
    T01 = T1
    T02 = T01.dot(T2)
    T03 = T02.dot(T3)
    T04 = T03.dot(T4)
    T05 = T04.dot(T5)
    T06 = T05.dot(T6)
    return T01, T02, T03, T04, T05, T06
    

def ur5_fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    _,_,_,_,_,T = transform(q)
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
    T01,T02,T03,T04,T05,T06 = transform(q)
    # Asignar posiciones
    pos[0:3,0] = T01[0:3,3]
    pos[0:3,1] = T02[0:3,3]
    pos[0:3,2] = T03[0:3,3]
    pos[0:3,3] = T04[0:3,3]
    pos[0:3,4] = T05[0:3,3]
    pos[0:3,5] = T06[0:3,3]
    return pos


def jacobian(q,n):
    if len(q) < 6:
        q = np.array([0,0,0,0,0,0])
    # Crear una matriz 3x6
    pos = np.zeros((3,6))
    T01,T02,T03,T04,T05,T06 = transform(q)
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


def jacobian_pose(q, delta=0.0000001):
    """
    @info Analytic jacobian for pose (orientation represented with quaternions)
    
    @param q: joint position [6x1]
    @param J: analytic jacobian [7x6]
    """
    J = np.zeros((7,6))
    # Initial homogeneous transformation (using q)
    T = ur5_fkine(q)
    Q = rot2quat(T[0:3,0:3])

    for i in range(6):
        dq      = copy(q)
        dq[i]   = dq[i] + delta
        dT      = ur5_fkine(dq)
        dQ      = rot2quat(dT[0:3,0:3])
        Jpos    = (dT[0:3,3] - T[0:3,3])/delta
        Jrot    = quatError(dQ, Q)/delta
        J[:,i] = np.concatenate((Jpos, Jrot), axis=0)
   
    return J



def rot2quat(R):
    
    #Convertir una matriz de rotacion en un cuaternion

    #Entrada:
    #  R -- Matriz de rotacion
    #Salida:
    #  Q -- Cuaternion [ew, ex, ey, ez]

    
    dEpsilon = 1e-6
    quat = 4*[0.,]

    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
    
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


def quatError(Qdes, Q):
    """
    Compute difference between quaterions.
    Inputs:
    ------
        - Qdes:     Desired quaternion
        - Q   :     Current quaternion
    Output:
    -------
        - Qe  :     Error quaternion    
    """
    we = Qdes[0]*Q[0] + np.dot(Qdes[1:4].transpose(),Q[1:4]) - 1
    e  = -Qdes[0]*Q[1:4] + Q[0]*Qdes[1:4] - np.cross(np.transpose(Qdes[1:4]), np.transpose(Q[1:4]))
    Qe = np.array([ we, e[0], e[1], e[2] ])

    return Qe  


def quatMult(q1, q2):
    quat = 4*[0.,]
    quat[0] =  q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3]
    quat[1] = -q1[0]*q2[1] + q2[0]*q1[1] - (q1[2]*q2[3] - q1[3]*q2[2])
    quat[2] = -q1[0]*q2[2] + q2[0]*q1[2] + (q1[1]*q2[3] - q1[3]*q2[1])
    quat[3] = -q1[0]*q2[3] + q2[0]*q1[3] - (q1[0]*q2[2] - q1[2]*q2[1])
    return np.array(quat)


def TF2xyzquat(T):
    
    #Convert a homogeneous transformation matrix into the a vector containing the
    #pose of the robot.

    #Input:
    #  T -- A homogeneous transformation
    #Output:
    #  X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
    #       is Cartesian coordinates and the last part is a quaternion
    
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
    med_f = alpha*np.array(med) + (1-alpha)*np.array(med_0)
    med_f = med_f.tolist()
    return med_f


def lidar_combination(med1, med2, ang1, ang2):
    x1 = med1*np.cos(ang1)
    y1 = med1*np.sin(ang1)+0.15
    
    x2 = med2*np.cos(ang2)
    y2 = med2*np.sin(ang2)-0.15
    
    x = np.append(x1,x2)
    y = np.append(y1,y2)
    
    ang = np.linspace(np.pi,-np.pi,len(y))
    ang = np.linspace(0,2*np.pi,len(y))
    med = np.power(x*x+y*y,0.5)
    med = np.clip(med,0,3)
    return med, ang


def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    med_f = ret[0:] / n
    med_f[0:n-1] =  a[0:n-1]
    return med_f
    
    
def person_pos_xy(med, ang, x_person_0):
    """
    Calcula la posición de la persona con respecto al robot
    """
    x_person = x_person_0

    pos,_ = find_peaks(-med,distance=10)
    if len(pos) == 2:
        med_p = med[pos]
        ang_p = ang[pos]
    
        r1 = med_p[0]+0.1
        th1 = ang_p[0]
        r2 = med_p[1]+0.1
        th2 = ang_p[1]
    
        x1 = np.array([r1*np.cos(th1), r1*np.sin(th1), 0, 1])
        x2 = np.array([r2*np.cos(th2), r2*np.sin(th2), 0, 1])
    
        x_person = (x1+x2)/2
        x_person = np.array([x_person[0], x_person[1], x_person[2], x_person[0], x_person[1], x_person[2]+1.7])
    
    return x_person


def person_position(med, ang):
    pos,_ = find_peaks(-med,height=-2.5,distance=10)
    med_p = med[pos] + 0.1
    ang_p = ang[pos]
        
    person = len(med_p)//2
    r = np.empty(person)
    th = np.empty(person)
       
    for i in range(person):
        r[i]  = (med_p[2*i]+med_p[2*i+1])/2
        th[i] = (ang_p[2*i]+ang_p[2*i+1])/2
        
    person_x = r*np.cos(th)
    person_y = r*np.sin(th)
    
    return person_x, person_y
        
    
def sat(x):
    sat_x = x
    for i in range(len(x)):
        if np.absolute(x[i]) <= 1:
            sat_x[i] = x[i]
        else:
            sat_x[i] = np.tanh(x[i])    
    return sat_x    
    
    
def ley_adap(M,P,dB,alpha):
    
    dM = M
    for i in range(6):
        if M[i,i] >= P[i,i]:
            dM[i,i] += -alpha*dB[i]
        else:
            dM[i,i] = P[i,i]
    return dM
    
    
def q_lim(q):
    q[0] = np.clip(q[0],-pi,pi)
    q[1] = np.clip(q[1],-pi,0)
    q[2] = np.clip(q[2],0,pi/2)
    q[3] = np.clip(q[3],-5*pi/4,pi/4)
    q[4] = np.clip(q[4],-pi,0)
    q[5] = np.clip(q[5],-2*pi,2*pi)
    return q
    

def QR_min(capsule_R, capsule_O):
    
    p1, u1, rad1 = capsule_R.read_variables()
    p1 = np.array(p1); u1 = np.array(u1); rad1 = np.array([rad1])
    s1 = u1 - p1
    
    p2, u2, rad2 = capsule_O.read_variables()
    p2 = np.array(p2); u2 = np.array(u2); rad2 = np.array([rad2])
    #print(p2, u2)
    s2 = u2 - p2
    
    A = np.array([[s2[0], -s1[0]],
                  [s2[1], -s1[1]],
                  [s2[2], -s1[2]]])
    y = np.array([[p2[0] - p1[0]],
                  [p2[1] - p1[1]],
                  [p2[2] - p1[2]]])
    
    Q,R = np.linalg.qr(A)
    
    n1 = np.array([[0], [1]])
    n2 = np.array([[1], [1]])
    n3 = np.array([[1], [0]])
    n4 = np.array([[0], [0]])
    
    n1 = R.dot(n1) + (Q.T).dot(y)
    n2 = R.dot(n2) + (Q.T).dot(y)
    n3 = R.dot(n3) + (Q.T).dot(y)
    n4 = R.dot(n4) + (Q.T).dot(y)
    N = [[n1[0][0], n2[0][0], n3[0][0], n4[0][0], n1[0][0]],
         [n1[1][0], n2[1][0], n3[1][0], n4[1][0], n1[1][0]]]
    
    m4 = n1[0] - n1[1]*R[0][1]/R[1][1]
    m3 = m4 + R[0][0]
    
    if n4[1]*n1[1] < 0 and m4*m3 < 0:
        u_min = np.array([0,0])
    else:
        for i in range(4):
            punto_1 = np.array([N[0][i],N[1][i]])
            punto_2 = np.array([N[0][i+1],N[1][i+1]])
            c = segment_min(punto_1, punto_2)
            if i == 0:
                u_min = c
            else:
                if np.linalg.norm(u_min) > np.linalg.norm(c):
                    u_min = c
    
    d_min = np.dot(u_min.T,u_min) + np.dot(y.T,y) - (y.T).dot(Q).dot(Q.T).dot(y)
    d_min = np.power(d_min,0.5) - rad1 - rad2
    
    x_min = np.linalg.inv(R).dot(u_min-np.dot(Q.T,y))
    v1 = np.array([p1]).T + A.dot(np.array([[0, 0],[0, -1]])).dot(x_min)
    v1 = np.array([v1[0][0],v1[1][0],v1[2][0]])
    v2 = np.array([p2]).T + A.dot(np.array([[1, 0],[0, 0]])).dot(x_min)
    v2 = np.array([v2[0][0],v2[1][0],v2[2][0]])
    x_min = np.array([x_min[0][0],x_min[1][0]])
    
    return u_min, d_min, x_min, v1, v2
        
def segment_min(a, b):
    ab = b - a
    
    u = (-a[0])*(ab[0]) + (-a[1])*(ab[1])
    u = u/((ab[0])*(ab[0]) + (ab[1])*(ab[1]))
    
    if u >= 1:
        c = b
    elif u <= 0:
        c = a    
    else:
        c = np.array([a[0]+u*(ab[0]), a[1]+u*(ab[1])])
    return np.array([c]).T
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
