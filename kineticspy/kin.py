import copy
import numpy as np
import math

pi = math.pi


# function to perform forward kinematics of open chain robot
def fwd(robobj, thetalist):

    rotmat = rotation_matrix(robobj, thetalist)

    robobj.coordmat = copy.deepcopy(robobj.initcoordmat)

    i = 1
    while i < robobj.jointno + 1:
        j = i - 1
        while j > -1:
            robobj.coordmat[i] = robobj.coordmat[i].__sub__(robobj.initcoordmat[j])
            robobj.coordmat[i] = rotmat[j].dot(robobj.coordmat[i])
            robobj.coordmat[i] = robobj.coordmat[i].__add__(robobj.initcoordmat[j])
            j = j - 1
        i = i + 1

    return robobj.coordmat


# create rotation matrix for each joint
def rotation_matrix(robobj, thetalist):

    i = 0
    rotmat = np.zeros((robobj.jointno, 3, 3))

    while i < robobj.jointno:
        rotmat[i][0] = [math.cos(thetalist[i]) + (robobj.jointaxismat[i][0]**2)*(1 - math.cos(thetalist[i])),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(1-math.cos(thetalist[i])) -
                         (robobj.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) +
                         (robobj.jointaxismat[i][1]*math.sin(thetalist[i])))]
        rotmat[i][1] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(1-math.cos(thetalist[i])) +
                         (robobj.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robobj.jointaxismat[i][1]**2)*(1 - math.cos(thetalist[i])),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) -
                         (robobj.jointaxismat[i][0]*math.sin(thetalist[i])))]
        rotmat[i][2] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) -
                         (robobj.jointaxismat[i][1]*math.sin(thetalist[i]))),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) +
                         (robobj.jointaxismat[i][0]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robobj.jointaxismat[i][2]**2)*(1 - math.cos(thetalist[i]))]

        i = i + 1

    return rotmat

#computes analytical solution for inverse
def inv(robobj, coord):

    [x, y, z] = coord
    thetalist = np.zeros(3)

    if math.sqrt(x**2 + y**2 + (z - robobj.lenmat[0][2])**2) > (robobj.lenmat[1][2] + robobj.lenmat[2][2]):
        print("Not Possible")

    if x == 0:
        thetalist[0] = 0
        return 0
    else:
        thetalist[0] = math.atan(y / x)
        if x < 0:
            thetalist[0] = thetalist[0] + pi

    d = math.sqrt((z - robobj.lenmat[0][2])**2 + x**2 + y**2)

    thetalist[2] = pi - math.acos((robobj.lenmat[1][2]**2 + robobj.lenmat[2][2]**2 - d**2)
                                  / (2*robobj.lenmat[1][2]*robobj.lenmat[2][2]))

    theta = math.asin(robobj.lenmat[2][2] * math.sin(thetalist[2]) / d)
    thetalist[1] = math.acos((z - robobj.lenmat[0][2]) / d) - theta

    return thetalist
