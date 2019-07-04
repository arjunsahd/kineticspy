import numpy as np
import copy
import math


def calc(robot, thetalist):

    i = 0
    rotmat = np.zeros((robot.jointno, 3, 3))

    while i < robot.jointno:
        rotmat[i][0] = [math.cos(thetalist[i]) + (robot.jointaxismat[i][0]**2)*(1 - math.cos(thetalist[i])),
                        ((robot.jointaxismat[i][0]*robot.jointaxismat[i][1])*(1-math.cos(thetalist[i])) - (robot.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        ((robot.jointaxismat[i][0]*robot.jointaxismat[i][2])*(1-math.cos(thetalist[i])) + (robot.jointaxismat[i][1]*math.sin(thetalist[i])))]
        rotmat[i][1] = [((robot.jointaxismat[i][0]*robot.jointaxismat[i][1])*(1-math.cos(thetalist[i])) + (robot.jointaxismat[i][2]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robot.jointaxismat[i][1]**2)*(1 - math.cos(thetalist[i])),
                        ((robot.jointaxismat[i][1]*robot.jointaxismat[i][2])*(1-math.cos(thetalist[i])) - (robot.jointaxismat[i][0]*math.sin(thetalist[i])))]
        rotmat[i][2] = [((robot.jointaxismat[i][0]*robot.jointaxismat[i][2])*(1-math.cos(thetalist[i])) - (robot.jointaxismat[i][1]*math.sin(thetalist[i]))),
                        ((robot.jointaxismat[i][1]*robot.jointaxismat[i][2])*(1-math.cos(thetalist[i])) + (robot.jointaxismat[i][0]*math.sin(thetalist[i]))),
                        math.cos(thetalist[i]) + (robot.jointaxismat[i][2]**2)*(1 - math.cos(thetalist[i]))]

        i = i + 1

    robot.coordmat = copy.deepcopy(robot.initcoordmat)

    i = 1
    while i < robot.jointno + 1:
        j = i - 1
        while j > -1:
            coordmatcp = copy.deepcopy(robot.coordmat)
            robot.coordmat[i][0] = rotmat[j][0][0]*coordmatcp[i][0] + rotmat[j][0][1]*coordmatcp[i][1] + rotmat[j][0][2]*coordmatcp[i][2]
            robot.coordmat[i][1] = rotmat[j][1][0]*coordmatcp[i][0] + rotmat[j][1][1]*coordmatcp[i][1] + rotmat[j][1][2]*coordmatcp[i][2]
            robot.coordmat[i][2] = rotmat[j][2][0]*coordmatcp[i][0] + rotmat[j][2][1]*coordmatcp[i][1] + rotmat[j][2][2]*coordmatcp[i][2]
            j = j - 1
        i = i + 1
