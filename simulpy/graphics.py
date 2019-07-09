import numpy as np
import plotly
import plotly.graph_objs as go
import copy
from simulpy.robobj import robot


def plot_layout(robobj):

    #mlen = max(robobj.initcoordmat[robobj.jointno-1])

    layout = go.Layout(

                        scene = dict(
                        xaxis = dict(
                            range = [-6, 6],),
                        yaxis = dict(
                            range = [-6, 6],),
                        zaxis = dict(
                            range = [-6, 6],),),
                      )
    return layout


def plot_initiatilize(robobj, positionmatrix):

    plotly.offline.init_notebook_mode(connected=True)

    i = 0
    initj = 0

    lenx = len(np.linspace(positionmatrix[i][0], positionmatrix[i+1][0]))
    x_array = np.zeros(lenx*3)
    y_array = np.zeros(lenx*3)
    z_array = np.zeros(lenx*3)

    while i < robobj.jointno:
        x = np.linspace(positionmatrix[i][0], positionmatrix[i+1][0])
        y = np.linspace(positionmatrix[i][1], positionmatrix[i+1][1])
        z = np.linspace(positionmatrix[i][2], positionmatrix[i+1][2])
        lenx = len(x)
        j = copy.deepcopy(initj)

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

    jointx = np.zeros(robobj.jointno + 1)
    jointy = np.zeros(robobj.jointno + 1)
    jointz = np.zeros(robobj.jointno + 1)

    while i < robobj.jointno + 1:
        jointx[i] = positionmatrix[i][0]
        jointy[i] = positionmatrix[i][1]
        jointz[i] = positionmatrix[i][2]
        i = i + 1

    joints = go.Scatter3d(
        x=jointx, y=jointy, z=jointz,
        marker=dict(
            size=5,
        ),
        line=dict(
            color='#ff7f0e',
            width=2
        )
    )

    robopos = robot.Robopos(armvec, joints)

    return robopos


def trajectory_initialise(robobj, trajectorymat):

    nopoints = len(trajectorymat)

    x_array = np.zeros(nopoints)
    y_array = np.zeros(nopoints)
    z_array = np.zeros(nopoints)

    i = 0
    while i < nopoints:
        x_array[i] = trajectorymat[i][robobj.jointno][0]
        y_array[i] = trajectorymat[i][robobj.jointno][1]
        z_array[i] = trajectorymat[i][robobj.jointno][2]
        i = i + 1

    trac = go.Scatter3d(
        x=x_array, y=y_array, z=z_array,
        marker=dict(
            size=5,
        ),
        line=dict(
            color='#ff7f0e',
            width=2
        )
    )

    return trac


def plotcurrpos(robobj):

    robopos = plot_initiatilize(robobj, robobj.coordmat)
    data =[robopos.armvec, robopos.joints]
    layout = plot_layout(robobj)
    fig = go.Figure(data=data, layout=layout)
    plotly.offline.iplot(fig)


def tracplot(robobj, trajectorymat, opt):

    plotly.offline.init_notebook_mode(connected=True)

    if opt == 1:

        robopos1 = plot_initiatilize(robobj, trajectorymat[0])
        robopos2 = plot_initiatilize(robobj, trajectorymat[len(trajectorymat) - 1])
        trac = trajectory_initialise(robobj, trajectorymat)
        layout = plot_layout(robobj)
        data = [robopos1.armvec, robopos1.joints, robopos2.armvec, robopos2.joints, trac]
        fig = go.Figure(data=data, layout=layout)
        plotly.offline.iplot(fig)

    else:

        trac = trajectory_initialise(robobj, trajectorymat)
        data = [trac]
        layout = plot_layout(robobj)
        fig = go.Figure(data=data, layout=layout)
        plotly.offline.iplot(fig)


