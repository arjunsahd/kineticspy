import numpy as np
import math
import plotly.graph_objs as go
from itertools import product


def jacobmat(robobj, theta):

    jacob = np.zeros(shape=(3,3))

    l2 = robobj.lenmat[1][2]
    l3 = robobj.lenmat[2][2]

    jacob[0][0] = -l2*math.sin(theta[0])*math.sin(theta[1]) - l3*math.sin(theta[0])*math.sin(theta[1] + theta[2])
    jacob[0][1] = l2*math.cos(theta[0])*math.cos(theta[1]) + l3*math.cos(theta[0])*math.cos(theta[1] + theta[2])
    jacob[0][2] = l3*math.cos(theta[0])*math.cos(theta[1] + theta[2])

    jacob[1][0] = l2*math.cos(theta[0])*math.sin(theta[1]) + l3*math.cos(theta[0])*math.sin(theta[1] + theta[2])
    jacob[1][1] = l2*math.sin(theta[0])*math.cos(theta[1]) + l3*math.sin(theta[0])*math.cos(theta[1] + theta[2])
    jacob[1][2] = l3*math.sin(theta[0])*math.cos(theta[1] + theta[2])

    jacob[2][0] = 0
    jacob[2][1] = l3*math.sin(theta[1] + theta[2]) - l2*math.sin(theta[1])
    jacob[2][2] = l3*math.sin(theta[1] + theta[2])

    return jacob


def vel_mani(robobj, orientation, max_joint_vel):

    vel_joint1 = np.linspace(-max_joint_vel[0], max_joint_vel[0], 2)
    vel_joint2 = np.linspace(-max_joint_vel[1], max_joint_vel[1], 2)
    vel_joint3 = np.linspace(-max_joint_vel[2], max_joint_vel[2], 2)

    b = vel_joint1, vel_joint2, vel_joint3
    mat = list(product(*b))

    siz = len(mat)
    i = 0

    elipx = np.zeros(siz)
    elipy = np.zeros(siz)
    elipz = np.zeros(siz)

    while i < siz:
        elip = jacobcalc(robobj, orientation, mat[i])
        elipx[i] = elip[0]
        elipy[i] = elip[1]
        elipz[i] = elip[2]
        i = i + 1

    end_velocity = go.Scatter3d(x=elipx, y=elipy, z=elipz,)

    data = [end_velocity]

    return data
