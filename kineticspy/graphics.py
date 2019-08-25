import numpy as np
import plotly
import plotly.graph_objs as go
import copy


# return the layout to be used while plotting
def plot_layout():

    # mlen = max(robobj.initcoordmat[robobj.jointno-1])

    layout = go.Layout(
                        scene=dict(
                            xaxis=dict(range=[-6, 6],),
                            yaxis=dict(range=[-6, 6],),
                            zaxis=dict(range=[-6, 6],),),
                      )
    return layout


def plot(data):
    plotly.offline.init_notebook_mode(connected=True)
    layout = plot_layout()
    fig = go.Figure(data=data, layout=layout)
    return fig


# initialize the plot for robot arm and return a Robopos(data type)
def position(robobj, positionmatrix):

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
    data = [armvec, joints]

    return data


# plot the current position of the robot
def current_position(robobj):
    data = position(robobj, robobj.coordmat)
    return data
