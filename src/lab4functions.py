#!/usr/bin/env python
# encoding: utf-8

import numpy as np
from copy import copy



def dh(d, th, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.

    """

    cth = np.cos(th);    sth = np.sin(th)
    ca = np.cos(alpha);  sa = np.sin(alpha)
    T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    
     

         #la tabla del DH   q(0),q(1)
    return T
    
    

def fkine_ur5(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]

    """
    # Longitudes (en metros)

    # Matrices DH (completar)
    T1 = dh(0.0892,q[0],0,np.pi/2)
    T2 = dh(0,q[1]+np.pi,0.425,0)
    T3 = dh(0,q[2],0.392,0)
    T4 = dh(0.1093,np.pi+q[3],0,np.pi/2)
    T5 = dh(0.09475,np.pi+q[4],0,np.pi/2)
    T6 = dh(0.0825,q[5],0,0)

    # Efector final con respecto a la base
    
    T = np.dot(T1,T2)
    
    T = np.dot(T,T3)
    
    T = np.dot(T,T4)
    
    T = np.dot(T,T5)
    
    T = np.dot(T,T6)
  
     #multiplicacion de todas las Tś
    return T


def jacobian_ur5(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]

    """
    # Alocacion de memoria
    J = np.zeros((3,6))
    # Transformacion homogenea inicial (usando q)

    TH=fkine_ur5(q)
    TH_aux=TH
    # Iteracion para la derivada de cada columna
    for i in xrange(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i]=dq[i]+delta
        # Transformacion homogenea luego del incremento (q+dq)
        TH_inc=fkine_ur5(dq)
        TH_delta2=np.array(TH_inc)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        for k in range(3):
            J[k,i]=(TH_delta2[k,3]-TH_aux[k,3])/delta
    return J


def ikine_ur5(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuración articular inicial de q0. 

    """
    epsilon  = 0.001
    max_iter = 1000
    delta    = 0.00001

    q  = copy(q0)
    for i in range(max_iter):
        # Main loop
        F_ini=np.array(fkine_ur5(q))
        dif=(jacobian_ur5(q,delta))
        error=xdes-F_ini[0:3,3]
        q=q+np.dot(np.linalg.pinv(dif),error)
        if(np.linalg.norm(error)<epsilon):
            break
        
    
    return q


