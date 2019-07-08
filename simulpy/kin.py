import copy
import numpy as np
import math

def fwd(robobj, thetalist):

    rotmat = rotation_matrix(robobj,thetalist)

    robobj.coordmat = copy.deepcopy(robobj.initcoordmat)

    i = 1
    while i < robobj.jointno + 1:
        j = i - 1
        while j > -1:
            coordmatcp = copy.deepcopy(robobj.coordmat)
            robobj.coordmat[i][0] = rotmat[j][0][0]*coordmatcp[i][0] + rotmat[j][0][1]*coordmatcp[i][1] + rotmat[j][0][2]*coordmatcp[i][2]
            robobj.coordmat[i][1] = rotmat[j][1][0]*coordmatcp[i][0] + rotmat[j][1][1]*coordmatcp[i][1] + rotmat[j][1][2]*coordmatcp[i][2]
            robobj.coordmat[i][2] = rotmat[j][2][0]*coordmatcp[i][0] + rotmat[j][2][1]*coordmatcp[i][1] + rotmat[j][2][2]*coordmatcp[i][2]
            j = j - 1
        i = i + 1

def rotation_matrix(robobj, thetalist):

    i = 0
    rotmat = np.zeros((robobj.jointno, 3, 3))

    while i < robobj.jointno:
        rotmat[i][0] = [math.cos(thetalist[i]) + (robobj.jointaxismat[i][0]**2)*(1 - math.cos(thetalist[i])),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(1-math.cos(thetalist[i])) - (robobj.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) + (robobj.jointaxismat[i][1]*math.sin(thetalist[i])))]
        rotmat[i][1] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(1-math.cos(thetalist[i])) + (robobj.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robobj.jointaxismat[i][1]**2)*(1 - math.cos(thetalist[i])),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) - (robobj.jointaxismat[i][0]*math.sin(thetalist[i])))]
        rotmat[i][2] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) - (robobj.jointaxismat[i][1]*math.sin(thetalist[i]))),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(1-math.cos(thetalist[i])) + (robobj.jointaxismat[i][0]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robobj.jointaxismat[i][2]**2)*(1 - math.cos(thetalist[i]))]

        i = i + 1

    return rotmat

def rotation_matrix_dif(robobj, thetalist)

    i = 0
    rotmatdif = np.zeros((robobj.jointno, 3, 3))

    while i < robobj.jointno:
        rotmatdif[i][0] = [-math.sin(thetalist[i]) + (robobj.jointaxismat[i][0]**2)*(math.sin(thetalist[i])),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(math.sin(thetalist[i])) - (robobj.jointaxismat[i][2]*math.cos(thetalist[i]))),
                        ((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) + (robobj.jointaxismat[i][1]*math.cos(thetalist[i])))]
        rotmatdif[i][1] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][1])*(math.sin(thetalist[i])) + (robobj.jointaxismat[i][2]*math.cos(thetalist[i]))),
                        -math.sin(thetalist[i]) + (robobj.jointaxismat[i][1]**2)*(math.sin(thetalist[i])),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) - (robobj.jointaxismat[i][0]*math.cos(thetalist[i])))]
        rotmatdif[i][2] = [((robobj.jointaxismat[i][0]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) - (robobj.jointaxismat[i][1]*math.cos(thetalist[i]))),
                        ((robobj.jointaxismat[i][1]*robobj.jointaxismat[i][2])*(math.sin(thetalist[i])) + (robobj.jointaxismat[i][0]*math.cos(thetalist[i]))),
                        math.sin(thetalist[i]) + (robobj.jointaxismat[i][2]**2)*(1 - math.sin(thetalist[i]))]
        i = i + 1

    return rotmatdif


def jacobian(robobj, thetalist):

    rotmat = rotation_matrix(robobj,thetalist)
    rotmatdif = rotation_matrix_dif(robobj,thetalist)

