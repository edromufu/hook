import numpy as np
from numpy import pi

m = 0.160
n = 0.094
q = 0.05

def angleConv(ang):
    ang = ang%(2*pi)

    if (ang >= pi):
        ang -= 2*pi

    return ang
        
def ik(x, z, theta):
    
    x1 = x - q*np.cos(theta)
    z1 = z + q*np.sin(theta)
    p = np.sqrt(x1**2 + z1**2)

    cosBeta = (p**2 - m**2 - n**2)/(2*m*n)
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

    xm = shoulderPositionX + m * np.sin(alfa)
    zm = shoulderPositionZ + m * np.cos(alfa)
    xn = xm + n * np.sin(alfa + beta)
    zn = zm + n * np.cos(alfa + beta)
    xq = xn + q * np.sin(alfa + beta + theta)
    zq = zn + q * np.cos(alfa + beta + theta)

    return [xq, zq]

def callIK(ikMotorsPosition, dx, dz, dtheta):
    [x0, z0] = fk(ikMotorsPosition)

    theta = np.sum(ikMotorsPosition) - (pi/2)
    
    try:
        output = ik(x0+dx, z0+dz, theta+dtheta)
    except Exception as e:
        print('Retornando posição original.')
        output = ikMotorsPosition

    return output