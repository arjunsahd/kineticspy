import numpy as np
from simulpy import fwdkin
import matplotlib.pyplot as plt


def calc(robot, thetainit, thetafinal, nopoints):

    thetamat = np.zeros((nopoints + 2, robot.jointno))
    thetamat[0] = thetainit

    i = 1
    while i < nopoints+2:
        j = 0
        while j < robot.jointno:
            thetamat[i][j] = round(thetainit[j] + i*thetafinal[j]/(nopoints+1), 2)
            j = j + 1
        i = i + 1

    i = 0
    trajectorymat = np.zeros((nopoints+2, robot.jointno + 1, 3))
    while i < nopoints + 2:
        fwdkin.calculate(robot, thetamat[i])
        trajectorymat[i] = robot.coordmat
        i = i + 1

    return trajectorymat


def plot(robot, trajectorymat, nopoints,opt):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    if(opt == 1):
        i = 0
        while i < robot.jointno:
            x = np.linspace(trajectorymat[0][i][0], trajectorymat[0][i+1][0])
            y = np.linspace(trajectorymat[0][i][1], trajectorymat[0][i+1][1])
            z = np.linspace(trajectorymat[0][i][2], trajectorymat[0][i+1][2])
            ax.plot(x, y, z, color='blue')
            i = i + 1

        i = 0
        while i < robot.jointno + 1:
            ax.scatter(trajectorymat[0][i][0], trajectorymat[0][i][1], trajectorymat[0][i][2], color='red')
            i = i + 1

        i = 0
        while i < robot.jointno:
            x = np.linspace(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i+1][0])
            y = np.linspace(trajectorymat[nopoints + 1][i][1], trajectorymat[nopoints + 1][i+1][1])
            z = np.linspace(trajectorymat[nopoints + 1][i][2], trajectorymat[nopoints + 1][i+1][2])
            ax.plot(x, y, z, color='blue')
            i = i + 1

        i = 0
        while i < robot.jointno + 1:
            ax.scatter(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i][1], trajectorymat[nopoints + 1][i][2], color='red')
            i = i + 1

    x_array = np.zeros(nopoints + 2)
    y_array = np.zeros(nopoints + 2)
    z_array = np.zeros(nopoints + 2)

    i = 0
    while i < nopoints + 2:
        x_array[i] = trajectorymat[i][robot.jointno][0]
        y_array[i] = trajectorymat[i][robot.jointno][1]
        z_array[i] = trajectorymat[i][robot.jointno][2]
        i = i + 1

    ax.plot(x_array, y_array, z_array, color='black')

    maximumpos = np.max(robot.initpos)
    ax.set_xlim3d(-maximumpos, maximumpos)
    ax.set_ylim3d(-maximumpos, maximumpos)
    ax.set_zlim3d(0, maximumpos)
    plt.show()
