import copy
import numpy as np
import math


def fwd(robobj, thetalist):

    rotmat = rotation_matrix(robobj, thetalist)

    robobj.coordmat = copy.deepcopy(robobj.initcoordmat)

    i = 1
    while i < robobj.jointno + 1:
        j = i - 1
        while j > -1:
            robobj.coordmat[i] = robobj.coordmat[i].__sub__(robobj.initcoordmat[j])
            coordmatcp = copy.deepcopy(robobj.coordmat)
            robobj.coordmat[i][0] = rotmat[j][0][0]*coordmatcp[i][0] + rotmat[j][0][1]*coordmatcp[i][1] + \
                                    rotmat[j][0][2]*coordmatcp[i][2]
            robobj.coordmat[i][1] = rotmat[j][1][0]*coordmatcp[i][0] + rotmat[j][1][1]*coordmatcp[i][1] + \
                                    rotmat[j][1][2]*coordmatcp[i][2]
            robobj.coordmat[i][2] = rotmat[j][2][0]*coordmatcp[i][0] + rotmat[j][2][1]*coordmatcp[i][1] + \
                                    rotmat[j][2][2]*coordmatcp[i][2]
            robobj.coordmat[i] = robobj.coordmat[i].__add__(robobj.initcoordmat[j])
            j = j - 1
        i = i + 1

    return robobj.coordmat

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


def rotation_matrix_dif(robobj, thetalist):

    i = 0
    rotmatdif = np.zeros((robobj.jointno, 3, 3))

    while i < robobj.jointno:
        rotmatdif[i][0] = [-math.sin(thetalist[i]) + (robobj.jointaxismat[i][0]**2)*(math.sin(thetalist[i])),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(math.sin(thetalist[i])) -
                         (robobj.jointaxismat[i][2]*math.cos(thetalist[i]))),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) +
                         (robobj.jointaxismat[i][1]*math.cos(thetalist[i])))]
        rotmatdif[i][1] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(math.sin(thetalist[i])) +
                            (robobj.jointaxismat[i][2]*math.cos(thetalist[i]))),
                        -math.sin(thetalist[i]) + (robobj.jointaxismat[i][1]**2)*(math.sin(thetalist[i])),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) -
                         (robobj.jointaxismat[i][0]*math.cos(thetalist[i])))]
        rotmatdif[i][2] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) -
                            (robobj.jointaxismat[i][1]*math.cos(thetalist[i]))),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) +
                         (robobj.jointaxismat[i][0]*math.cos(thetalist[i]))),
                        math.sin(thetalist[i]) + (robobj.jointaxismat[i][2]**2)*(1 - math.sin(thetalist[i]))]
        i = i + 1

    return rotmatdif


def jacobian(robobj, thetalist):

    rotmat = rotation_matrix(robobj, thetalist)
    rotmatdif = rotation_matrix_dif(robobj, thetalist)

    jparts = np.zeros((robobj.nopoint, 3, 3))

    i = 0
    while i < robobj.jointno:
        jparts[i] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        i = i + 1

    i = 0
    while i < robobj.jointno:
        k = 0
        while k < robobj.jointno:
            if k == i:
                jparts[i] = np.dot(rotmatdif[k], jparts[i])
            else:
                jparts[i] = np.dot(rotmat[k], jparts[i])
            k = k + 1
        i = i + 1

    jacob = np.zeros(3, 3)
    i = 0
    while i < robobj.jointno:
        jacob = jacob.__add__(jparts[i])
        i = i + 1

    return jacob
