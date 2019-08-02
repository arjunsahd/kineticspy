import numpy as np
import math
import copy


class Robot(object):

    def __init__(self, jointno, jointaxismat, lenmat):
        self.jointno = jointno
        self.jointaxismat = jointaxismat
        self.lenmat = lenmat

        lenvecmat = np.zeros((jointno, 4))
        i = 0
        while i < jointno:
            mag = math.sqrt(lenmat[i][0]**2 + lenmat[i][1]**2 + lenmat[i][2]**2)
            lenvecmat[i] = [lenmat[i][0]/mag, lenmat[i][1]/mag, lenmat[i][2]/mag, mag]
            i = i + 1

        self.initcoordmat = np.zeros((jointno + 1, 3))
        i = 1
        while i < jointno+1:
            self.initcoordmat[i, 0] = self.initcoordmat[i-1, 0] + lenvecmat[i-1, 0]*lenvecmat[i-1, 3]
            self.initcoordmat[i, 1] = self.initcoordmat[i-1, 1] + lenvecmat[i-1, 1]*lenvecmat[i-1, 3]
            self.initcoordmat[i, 2] = self.initcoordmat[i-1, 2] + lenvecmat[i-1, 2]*lenvecmat[i-1, 3]
            i = i + 1

        self.coordmat = copy.deepcopy(self.initcoordmat)

class Robopos(object):

     def __init__(self, armvec, joints):
         self.armvec = armvec
         self.joints = joints

class body(object):

    def __init__(self, data1, data2, data3, data4, data5, data6):
        self.data1 = data1
        self.data2 = data2
        self.data3 = data3
        self.data4 = data4
        self.data5 = data5
        self.data6 = data6
