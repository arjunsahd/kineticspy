import numpy as np
import math
import plotly
import plotly.graph_objs as go
from numpy import linalg as LA
from numpy import sin, cos, pi

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


def vel_mani_elip(robobj, orientation):

    jacob = jacobmat(robobj, orientation)
    a = np.dot(jacob, np.transpose(jacob))
    eigen_val, eigen_vec = LA.eig(a)

    phi = np.linspace(0, 2*pi)
    theta = np.linspace(-pi/2, pi/2)
    phi, theta=np.meshgrid(phi, theta)

    x = cos(theta) * sin(phi) * eigen_val[0]
    y = cos(theta) * cos(phi) * eigen_val[1]
    z = sin(theta) * eigen_val[2]

    plotly.offline.init_notebook_mode(connected=True)

    elip = go.Mesh3d(alphahull = 0,
                    x= x.flatten(),
                    y= y.flatten(),
                    z= z.flatten(),
                    )

    data = [elip]

    return data
