import numpy as np
from numpy import pi

def angleConv(ang):
    if (ang >= 0 and ang <= pi/2):
      return ang
    else:
      return ang - pi*2
        
def ik(x, z, theta):
    m = 0.160
    n = 0.094
    q = 0.17935

    x1 = x - q*np.cos(theta)
    z1 = z + q*np.sin(theta)
    p = np.sqrt(x1*x1 + z1*z1)

    cosBeta = (p*p - m*m - n*n)/(2*m*n)
    beta = np.arccos(cosBeta)
    gama = pi - beta
    sinphi = n/p*np.sin(gama)
    phi = np.arcsin(sinphi)

    if x1>0:
        delta = np.arctan(z1/x1)
    else:
        delta = np.arctan(z1/x1)+pi
    
    alfa = pi/2 - phi - delta
    thetaL = pi/2 - (alfa + beta) + theta

    return [angleConv(alfa), angleConv(beta), angleConv(thetaL)]

def fk(motorsPosition):
    shoulderPositionX = 0
    shoulderPositionZ = 0

    alfa = motorsPosition[0] #Rotação atual do shoulder
    beta = motorsPosition[1] #... do elbow
    theta = motorsPosition[2] #... do wrist

    xm = shoulderPositionX + m * sin(alfa)
    zm = shoulderPositionZ + m * cos(alfa)
    xn = xm + n * sin(alfa + beta)
    zn = zm + n * cos(alfa + beta)
    xq = xn + q * sin(alfa + beta + theta)
    zq = zn + q * cos(alfa + beta + theta)

    return [xq, zq]

def callIK(ikMotorsPosition, dx, dz, dtheta):
    [x0, z0] = fk(ikMotorsPosition)

    theta = np.sum(ikMotorsPosition) + pi/2

    return ik(x0+dx, z0+dz, theta+dtheta)
    