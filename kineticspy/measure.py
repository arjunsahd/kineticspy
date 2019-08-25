import numpy as np
import math
import plotly.graph_objs as go
from itertools import product


def jacobcalc(robobj, orientation, joint_vel):

    jacob = np.zeros(3)

    jacob[0] = -robobj.lenmat[1][2]*math.sin(orientation[0])*math.sin(orientation[1])*joint_vel[0]
    -robobj.lenmat[2][2]*math.sin(orientation[0])*math.sin(orientation[1] + orientation[2])*joint_vel[0]
    +robobj.lenmat[1][2]*math.cos(orientation[0])*math.cos(orientation[1])*joint_vel[1]
    +robobj.lenmat[2][2]*math.cos(orientation[0])*math.cos(orientation[1] + orientation[2])*joint_vel[1]
    +robobj.lenmat[2][2]*math.cos(orientation[0])*math.cos(orientation[1] + orientation[2])*joint_vel[2]

    jacob[1] = robobj.lenmat[1][2]*math.cos(orientation[0])*math.sin(orientation[1])*joint_vel[0]
    +robobj.lenmat[2][2]*math.cos(orientation[0])*math.sin(orientation[1] + orientation[2])*joint_vel[0]
    +robobj.lenmat[1][2]*math.sin(orientation[0])*math.cos(orientation[1])*joint_vel[1]
    +robobj.lenmat[2][2]*math.sin(orientation[0])*math.cos(orientation[1] + orientation[2])*joint_vel[1]
    +robobj.lenmat[2][2]*math.sin(orientation[0])*math.cos(orientation[1] + orientation[2])*joint_vel[2]

    jacob[2] = -robobj.lenmat[1][2]*math.sin(orientation[1])*joint_vel[1]
    -robobj.lenmat[2][2]*math.sin(orientation[1] + orientation[2])*joint_vel[1]
    -robobj.lenmat[2][2]*math.sin(orientation[1] + orientation[2])*joint_vel[2]

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
