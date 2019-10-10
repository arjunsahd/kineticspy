import numpy as np


def write(robobj, saving_location):

    for i in range(0, len(robobj.cmdmat)):
        robobj.cmdmat[i] = robobj.cmdmat[i] * 57.2958

    np.savetxt(saving_location, robobj.cmdmat, delimiter=",")
