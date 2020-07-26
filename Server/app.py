#!/usr/bin/env python3
"""File    : app.py
*  Author  : Mathieu Schnegg
*  Date    : 02.05.2020
*
* Description : Server used to compute position
* with the data provided by the device
*
* The computation of the position
* is based on the work of Basile Graf
*
* Modifications : Date / Author / Purpose
*
* Platform : Raspberry Pi
"""
import asyncio
import datetime
import random
import websockets
import json
import serial
import sys
import glob
from lfsr import LFSR
import sympy as sp
import numpy as np
import scipy.integrate as integrate
import scipy.optimize as opt
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
from math import sin,tan,atan

#============================================================
# Open serial port
ser = serial.Serial('/dev/ttyACM0',bytesize=8)
print(ser.name) 

#============================================================
# Variables
# LFSR polynomials : first one is x^17 + x^13 + x^12 + x^10 + x^7 + x^4 + x^2 + x^1 + 1
polys = [0x0001D258, 0x00017E04,
         0x0001FF6B, 0x00013F67,
         0x0001B9EE, 0x000198D1,
         0x000178C7, 0x00018A55,
         0x00015777, 0x0001D911,
         0x00015769, 0x0001991F,
         0x00012BD0, 0x0001CF73,
         0x0001365D, 0x000197F5,
         0x000194A0, 0x0001B279,
         0x00013A34, 0x0001AE41,
         0x000180D4, 0x00017891,
         0x00012E64, 0x00017C72,
         0x00019C6D, 0x00013F32,
         0x0001AE14, 0x00014E76,
         0x00013C97, 0x000130CB,
         0x00013750, 0x0001CB8D]

# Expressed using a 48 MHz clock
periods = [959000, 957000,
           953000, 949000,
           947000, 943000,
           941000, 939000,
           937000, 929000,
           919000, 911000,
           907000, 901000,
           893000, 887000]

endMarker = 0xFF
beam  = np.zeros((3,3), dtype=int)
fBeam = np.zeros((3,3), dtype=float)
rawBeam = np.zeros((3,2), dtype=int)
beamLFSR0 = np.zeros((3,2), dtype=int)
beamLFSR1 = np.zeros((3,2), dtype=int)

MASK = 0x1FFFF

sensorsCoordinates = np.array([[-0.0035,0,0.0093],
                               [0.0035,0,0.0093],
                               [0.0057,0.0057,0.0057],
                               [0,0.0093,0.0035],
                               [-0.0057,0.0057,0.0057],
                               [-0.0093,0.0035,0],
                               [-0.0093,-0.0035,0],
                               [-0.0057,-0.0057,0.0057],
                               [0,-0.0093,0.0035],
                               [0.0057,-0.0057,0.0057],
                               [0.0093,-0.0035,0],
                               [0.0093,0.0035,0],
                               [0.0057,0.0057,-0.0057],
                               [0,0.0093,-0.0035],
                               [-0.0057,0.0057,-0.0057],
                               [-0.0035,0,-0.0093],
                               [-0.0057,-0.0057,-0.0057],
                               [0,-0.0093,-0.0035],
                               [0.0057,-0.0057,-0.0057]])

x = 0
y = 0
z = 0
roll = 0
pitch = 0
yaw = 0
bat = 0
temp = 0

# Compute the first two LFSR
x = 2
u = np.zeros((2**17,x),dtype=int)
for i in range(0,x):
    lfsr = LFSR(polys[i])
    for j in range(0,2**17):
        u[j,i]=next(lfsr)
    print('LFSR ',i,' done')

lfsr0 = list(u[:,0])
lfsr1 = list(u[:,1])

# Quaternion
q0, q1, q2, q3 = sp.symbols('q0,q1,q2,q3')
q = sp.Matrix([q0, q1, q2, q3])

# Vector v in R3
v1, v2, v3 = sp.symbols('v1,v2,v3')
v = sp.Matrix([v1, v2, v3])

# Rotation matrix
R = sp.Matrix([
    [q0**2 + q1**2 - q2**2 - q3**2, 2*q1*q2 - 2*q0*q3, 2*q0*q2 + 2*q1*q3],
    [2*q1*q2 + 2*q0*q3, q0**2 - q1**2 + q2**2 - q3**2, -2*q0*q1 + 2*q2*q3],
    [-2*q0*q2 + 2*q1*q3, 2*q0*q1 + 2*q2*q3, q0**2 - q1**2 - q2**2 + q3**2]
    ])

Rv = R*v
dRvdq = Rv.jacobian(q)

# Make functions
R_f = sp.lambdify(((q),), R)
dRvdq_f = sp.lambdify(((q),(v),), dRvdq)

#============================================================
# Functions definition
""" Finds all serial ports and returns a list containing them
    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
"""
def list_ports() -> list:
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)    # Try to open a port
            s.close()                  # Close the port if sucessful
            result.append(port)        # Add to list of good ports
        except (OSError, Exception):   # If un sucessful
            pass
    return result

""" Loop to update the server
    - magic happen here -
    :returns:
        none 
"""
async def time(websocket, path):
    runNb = 0
    print("Server started")

    while True:   
        # Wait for data on serial port
        ser.flushInput()
        while ser.inWaiting() == 0:
            pass
        s = ser.read(30)    # Expected 30bytes of data
        ser.flushInput()

        if(s[29]==endMarker):   # Success retrieving data 

            bat = s[0]  # Get battery level in %
            temp = s[1] # Get temperature

            # Get IMU data
            roll = (s[2]+(s[3]<<8))/100
            pitch = round((s[4]+(s[5]<<8))/100 - 180,2)
            yaw   = round((s[6]+(s[7]<<8))/100 - 180,2)

            # Get slave number
            beam[0][0] = s[8]
            beam[1][0] = s[15]
            beam[2][0] = s[22]

            # Construct beam 17bits value
            rawBeam[0][0] = ((s[9]<<16)+(s[10]<<8)+s[11])&MASK
            rawBeam[0][1] = ((s[12]<<16)+(s[13]<<8)+s[14])&MASK
            rawBeam[1][0] = ((s[16]<<16)+(s[17]<<8)+s[18])&MASK
            rawBeam[1][1] = ((s[19]<<16)+(s[20]<<8)+s[21])&MASK
            rawBeam[2][0] = ((s[23]<<16)+(s[24]<<8)+s[25])&MASK
            rawBeam[2][1] = ((s[26]<<16)+(s[27]<<8)+s[28])&MASK

            print("Raw beam :")
            print(rawBeam)

            # Convert to LFSRs count value
            for i in range(3):
                for j in range(2):
                    try:
                        beamLFSR0[i][j]=lfsr0.index(rawBeam[i][j])
                        beamLFSR1[i][j]=lfsr1.index(rawBeam[i][j])

                    except ValueError:
                        print("Not in list")

            print("Sequence with LFSR 0 :")
            print(beamLFSR0)
            print("Sequence with LFSR 1 :")
            print(beamLFSR1)

            # Check which one has the minimum delta
            if(abs(beamLFSR0[0][0]-beamLFSR0[1][0])<abs(beamLFSR1[0][0]-beamLFSR1[1][0])):
                # Check beam sequence
                for i in range(3):
                    if(beamLFSR0[i][0]>beamLFSR0[i][1]):    # Inverse sequence
                        beam[i][1] = beamLFSR0[i][1]
                        beam[i][2] = beamLFSR0[i][0]
                    else :                              # Keep sequence
                        beam[i][1] = beamLFSR0[i][0]
                        beam[i][2] = beamLFSR0[i][1]
            else:
                # Check beam sequence
                for i in range(3):
                    if(beamLFSR1[i][0]>beamLFSR1[i][1]):    # Inverse sequence
                        beam[i][1] = beamLFSR1[i][1]
                        beam[i][2] = beamLFSR1[i][0]
                    else :                              # Keep sequence
                        beam[i][1] = beamLFSR1[i][0]
                        beam[i][2] = beamLFSR1[i][1]

            print("Final sequence :")
            print(beam)

            # Compute the angles
            alpha = np.zeros((3,1),dtype=float)
            beta = np.zeros((3,1),dtype=float)

            for i in range(3):
                fBeam[i][1] = ((beam[i][1] * 8.0) / periods[0]) * 2 * np.pi
                fBeam[i][2] = ((beam[i][2] * 8.0) / periods[0]) * 2 * np.pi

                p = 60/180*np.pi
                b = (fBeam[i][2] - fBeam[i][1]) - 120/180*np.pi

                alpha[i] = ((fBeam[i][1] + fBeam[i][2]) / 2) - np.pi;    # Azimuth
                beta[i] = atan(sin(b/2)/tan(p/2));                     # Elevation
                
            # Print angles (debug purpose)
            print("Azimuth   : ")
            print(alpha)
            print("Elevation : ")
            print(beta)

            # Charge object geometry according to the slave used
            v = np.array([sensorsCoordinates[beam[0][0]],
                          sensorsCoordinates[beam[1][0]],
                          sensorsCoordinates[beam[2][0]]])

            # Lighthouse position  
            p = np.array([[0.0,0.0,0.0]]).transpose()

            # First run, need arbitrary init data 
            if(runNb == 0):
                # Object center
                c = np.array([1.5,2.2,1.2])
                    
                # Quaternion 
                quat = Quaternion(axis=[1,1,0.5], angle = 1.22)
                q = quat.elements

                # Rotate
                w = np.matmul(R_f(q), v)
                    
                # Construct angles alpha and beta
                dist_p_cible =  np.array([c,c,c]).transpose() + w - np.concatenate((p,p,p), axis=1)

                # Matrix (vector) of r distances
                r = np.linalg.norm(dist_p_cible, axis=0)
                r=np.array([r]).transpose() # matrie Nx1

                runNb = 1

            zInit = vars2z(c, q, r)

            # Optimisiation under constraints
            # https://docs.scipy.org/doc/scipy/reference/optimize.html
            fMin = lambda zz : toMin(zz, p, v, alpha, beta)
            dfMin_z = lambda zz : dtoMin_z(zz, p, v, alpha, beta)
            cons = opt.NonlinearConstraint(constraint,0,0, jac=dconstraint_z)
            optSol = opt.minimize(fMin, zInit, constraints=cons, tol=1e-9, jac=dfMin_z)

            zOpt = optSol.x
            cOpt, qOpt, rOpt = z2vars(zOpt, alpha.shape[1])

            # Extract x,y,z coordinates
            x = round(cOpt[0],4)
            y = round(cOpt[1],4)
            z = round(cOpt[2],4)

            # Put data in json format
            data = {'x':x,'y':y,'z':z,'roll':roll,'pitch':pitch,'yaw':yaw,'bat':bat,'temp':temp}

            # Send data
            print_json = json.dumps(data)
            await websocket.send(print_json)

            # Save actual position to use as next initial condition
            c = cOpt
            quat = qOpt
            r = rOpt

# Vecteur directionel normé d'azimuth alpha et élévation beta
def vecn(alpha, beta):
    return np.array([
            np.cos(alpha) * np.cos(beta), 
            np.sin(alpha) * np.cos(beta), 
            np.sin(beta)])

# Vecteur d'erreur individuelle err[i,k]  pour chaque cible
def err(c, q, r, p, v, alpha, beta, i, k):
    R = R_f(q);
    n_ik = vecn(alpha[i,k], beta[i,k]);
    return c + np.matmul(R,v[:,i]) - p[:,k] - r[i,k] * n_ik;

# Dérivée de err[i,k] par rapport au vecteur c
def derr_c(c, q, r, p, v, alpha, beta, i, k):
    return np.identity(3)

# Dérivée de err[i,k] par rapport au coefficients r[j,s] 
def derr_r(c, q, r, p, v, alpha, beta, i, k, j, s):
    if i != j:
        return np.array([0,0,0]);
    if k != s:
        return np.array([0,0,0]);
    n_ik = -1.0 * vecn(alpha[i,k], beta[i,k]);
    return n_ik

# Dérivée de err[i,k] par rapport au quaternion q
def derr_q(c, q, r, p, v, alpha, beta, i, k):
    return dRvdq_f(q, v[:,i])

# Dérivées de  (1/2) * | err[i,k] |^2 par rapport au vecteur c
def derr2_c(c, q, r, p, v, alpha, beta, i, k):    
    return np.dot(
            err(c, q, r, p, v, alpha, beta, i, k),
            derr_c(c, q, r, p, v, alpha, beta, i, k)
            )
    
# Dérivées de  (1/2) * | err[i,k] |^2 par rapport au coefficients r[j,s] 
# On construit directement la matrice des dérivées
def derr2_r(c, q, r, p, v, alpha, beta, i, k):  
    res = np.zeros(alpha.shape);
    err_ik = err(c, q, r, p, v, alpha, beta, i, k);
    for j in range(r.shape[0]):
        for s in range(r.shape[1]):
            res[j,s] = np.dot(
                    err_ik,
                    derr_r(c, q, r, p, v, alpha, beta, i, k, j, s)
                    )
    return res
    
# Dérivées de  (1/2) * | err[i,k] |^2 par rapport au quaternion q
def derr2_q(c, q, r, p, v, alpha, beta, i, k):    
    return np.dot(
            err(c, q, r, p, v, alpha, beta, i, k),
            derr_q(c, q, r, p, v, alpha, beta, i, k)
            )

# Fonction de coût complète
def cost(c, q, r, p, v, alpha, beta):
    N = alpha.shape[0]
    K = alpha.shape[1]
    cst=0;
    for i in range(N):
        for k in range(K):
            err_ik = err(c, q, r, p, v, alpha, beta, i, k)
            cst += 0.5 * np.dot(err_ik, err_ik)
    cst /= (N*K)
    return cst

# Dérivées de la fonction de coût complète par rapport à toutes les inconnues 
# c, r et q. Les dérivées ont la même dimension que les inconnues respectives
# ( la fonction de coût étant scalaire)
def dcost(c, q, r, p, v, alpha, beta):
    N = alpha.shape[0]
    K = alpha.shape[1]
    dcst_c=0;
    dcst_r=0;
    dcst_q=0;
    for i in range(N):
        for k in range(K):
            dcst_c += derr2_c(c, q, r, p, v, alpha, beta, i, k)
            dcst_r += derr2_r(c, q, r, p, v, alpha, beta, i, k)
            dcst_q += derr2_q(c, q, r, p, v, alpha, beta, i, k)
    dcst_c /= (N*K)
    dcst_r /= (N*K)
    dcst_q /= (N*K)
    return dcst_c, dcst_r, dcst_q

# Optimisation
# variable z pour représenter l'ensemble de inconnues c, q et r
def vars2z(c, q, r):
    return np.concatenate((c.reshape(-1), q.reshape(-1), r.reshape(-1)))

def z2vars(z, K):
    c = z[0:3]
    q = z[3:7]
    r = z[7:]
    r = r.reshape((-1,K))
    return c, q, r

# Contrainte de la norm de q, exprimée sur z
def constraint(z):
    q = z[3:7]
    return np.dot(q,q) - 1.0

# Dérivée de la contrainte (facile: c'est 2*q)
def dconstraint_z(z):
    dcdq = np.zeros(z.shape);
    dcdq[3:7] = 2 * z[3:7]
    return dcdq

# fonction à minimiser exprimée en z
def toMin(z, p, v, alpha, beta):
    K = alpha.shape[1]
    c, q, r = z2vars(z, K)
    return cost(c, q, r, p, v, alpha, beta)

# dérivées de la fonction à minimiser par rapport à z
def dtoMin_z(z, p, v, alpha, beta):
    K = alpha.shape[1]
    c, q, r = z2vars(z, K)
    dcst_c, dcst_r, dcst_q = dcost(c, q, r, p, v, alpha, beta)    
    return vars2z(dcst_c, dcst_q, dcst_r)

#============================================================
# Main code
def main():
    print("Started")

    # Start and run the server forever
    start_server = websockets.serve(time, "0.0.0.0", 5678)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    # execute only if run as a script
    main()