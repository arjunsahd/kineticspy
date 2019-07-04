import numpy as np
import plotly
import plotly.graph_objs as go
import copy


def plotrobot(robot):

    plotly.offline.init_notebook_mode(connected=True)

    print(robot.coordmat)
    i = 0
    initj = 0
    while i < robot.jointno:
        x = np.linspace(robot.coordmat[i][0], robot.coordmat[i+1][0])
        y = np.linspace(robot.coordmat[i][1], robot.coordmat[i+1][1])
        z = np.linspace(robot.coordmat[i][2], robot.coordmat[i+1][2])
        lenx = len(x)
        j = copy.deepcopy(initj)

        if i == 0:
            x_array = np.zeros(3*lenx)
            y_array = np.zeros(3*lenx)
            z_array = np.zeros(3*lenx)

        while j < lenx + initj:
            x_array[j] = x[j-initj]
            y_array[j] = y[j-initj]
            z_array[j] = z[j-initj]
            j = j + 1

        i = i + 1
        initj = copy.deepcopy(j)

    armvec = go.Scatter3d(
        x=x_array, y=y_array, z=z_array,
        marker=dict(
            size=2,
        ),
        line=dict(
            color='#1f77b4',
            width=1
        )
    )

    i = 0

    jointx = np.zeros(robot.jointno + 1)
    jointy = np.zeros(robot.jointno + 1)
    jointz = np.zeros(robot.jointno + 1)

    while i < robot.jointno + 1:
        jointx[i] = robot.coordmat[i][0]
        jointy[i] = robot.coordmat[i][1]
        jointz[i] = robot.coordmat[i][2]
        i = i + 1

    joint = go.Scatter3d(
        x=jointx, y=jointy, z=jointz,
        marker=dict(
            size=5,
        ),
        line=dict(
            color='#ff7f0e',
            width=2
        )
    )

    data = [armvec, joint]
    fig = go.Figure(data=data, layout={})

    plotly.offline.iplot(fig)
