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
            robobj.coordmat[i] = rotmat[j].dot(robobj.coordmat[i])
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

    jparts = np.zeros((robobj.jointno, 3))
    jacob = np.zeros((robobj.jointno, 3))

    i = 0
    while i < robobj.jointno:
        jparts[i] = robobj.initcoordmat[i+1]
        i = i + 1

    i = 0
    while i < robobj.jointno:

        j = 0
        while j < robobj.jointno:
            k = j
            while k > -1:
                jparts[j] = jparts[j].__sub__(robobj.initcoordmat[k])
                if k == i:
                    jparts[j] = rotmatdif[k].dot(jparts[j])
                else:
                    jparts[j] = rotmat[k].dot(jparts[j])
                jparts[j] = jparts[j].__add__(robobj.initcoordmat[k])

                k = k - 1
            j = j + 1

        jacob = jacob.__add__(jparts)
        i = i + 1

    return jacob[robobj.jointno - 1]


def inv(robobj, pos, thetaguess, er):
    found = 0
    ind = robobj.jointno
    i = 0

    while found == 0:
        g1 = fwd(robobj,thetaguess)
        jac = jacobian(robobj,thetaguess)

        print(g1[ind])

        if math.sqrt( (g1[ind][0] - pos[0])**2 + (g1[ind][1] - pos[1])**2 + (g1[ind][2] - pos[2])**2) < er:
            found = 1
        elif i > 10:
            found = 2
        else:
            thetaguess[0] = thetaguess[0] + (g1[ind][0] - pos[0])/jac[0]
            thetaguess[1] = thetaguess[1] + (g1[ind][1] - pos[1])/jac[1]
            thetaguess[2] = thetaguess[2] + (g1[ind][2] - pos[2])/jac[2]
        print(i)
        i = i + 1

    return thetaguess
