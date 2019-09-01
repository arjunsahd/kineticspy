import numpy as np
import math
from . import kin
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

    eigen_val[0] = eigen_val[0]/3
    eigen_val[1] = eigen_val[1]/3
    eigen_val[2] = eigen_val[2]/3

    phi = np.linspace(0, 2*pi)
    theta = np.linspace(-pi/2, pi/2)
    phi, theta=np.meshgrid(phi, theta)

    X = cos(theta) * sin(phi) * eigen_val[0]
    Y = cos(theta) * cos(phi) * eigen_val[1]
    Z = sin(theta) * eigen_val[2]

    xt = eigen_vec[0][0]*X + eigen_vec[0][1]*Y + eigen_vec[0][2]*Z
    yt = eigen_vec[1][0]*X + eigen_vec[1][1]*Y + eigen_vec[1][2]*Z
    zt = eigen_vec[2][0]*X + eigen_vec[2][1]*Y + eigen_vec[2][2]*Z

    coordmat = kin.fwd(robobj,orientation)

    x = xt + coordmat[3][0]
    y = yt + coordmat[3][1]
    z = zt + coordmat[3][2]

    elip = go.Mesh3d(alphahull = 0,
                    x= x.flatten(),
                    y= y.flatten(),
                    z= z.flatten(),
                    opacity = 0.5
                    )

    data = [elip]

    print("Eigen Values are:")
    print(eigen_val)
    max_val = np.max(eigen_val)
    min_val = np.min(eigen_val)

    print("\nManipulability Index: \t")
    print(math.sqrt(max_val/min_val))

    return data
