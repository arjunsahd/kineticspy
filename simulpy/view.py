import numpy as np
import matplotlib.pyplot as plt


def plotrobot(robot):
    fig = plt.figure()
    ax = fig.gca(projection='3d')

    print(robot.coordmat)
    i = 0

    while i < robot.jointno:
        x = np.linspace(robot.coordmat[i][0], robot.coordmat[i+1][0])
        y = np.linspace(robot.coordmat[i][1], robot.coordmat[i+1][1])
        z = np.linspace(robot.coordmat[i][2], robot.coordmat[i+1][2])
        ax.plot(x, y, z)
        i = i + 1

    i = 0
    while i < robot.jointno + 1:
        ax.scatter(robot.coordmat[i][0], robot.coordmat[i][1], robot.coordmat[i][2], color = 'blue')
        i = i + 1

    ax.set_xlim3d(-6, 6)
    ax.set_ylim3d(-6, 6)
    ax.set_zlim3d(0, 6)
    plt.show()
