import numpy as np
from simulpy import kin


def calctrajectorymat(thetamat, robobj):

    i = 0
    nopoints = len(thetamat)
    trajectorymat = np.zeros((nopoints, robobj.jointno + 1, 3))

    while i < nopoints:
        trajectorymat[i] = kin.fwd(robobj, thetamat[i])
        i = i + 1

    return trajectorymat


def calctrac(robobj, thetainit, thetafinal, nopoints):

    thetamat = np.zeros((nopoints + 2, robobj.jointno))
    thetamat[0] = thetainit

    i = 1
    while i < nopoints + 2:
        j = 0
        while j < robobj.jointno:
            thetamat[i][j] = thetainit[j] + i*thetafinal[j]/(nopoints+1)
            j = j + 1
        i = i + 1

    return calctrajectorymat(thetamat, robobj)
