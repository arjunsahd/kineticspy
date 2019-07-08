import numpy as np
import plotly
import plotly.graph_objs as go
import copy


def plotrobot(robobj):

    data = plot_initiatilize(robobj)

    fig = go.Figure(data=data, layout={})
    plotly.offline.iplot(fig)


def plot_initiatilize(robobj):
    plotly.offline.init_notebook_mode(connected=True)

    print(robobj.coordmat)
    i = 0
    initj = 0

    lenx = len(np.linspace(robobj.coordmat[i][0], robobj.coordmat[i+1][0]))
    x_array = np.zeros(lenx*3)
    y_array = np.zeros(lenx*3)
    z_array = np.zeros(lenx*3)

    while i < robobj.jointno:
        x = np.linspace(robobj.coordmat[i][0], robobj.coordmat[i+1][0])
        y = np.linspace(robobj.coordmat[i][1], robobj.coordmat[i+1][1])
        z = np.linspace(robobj.coordmat[i][2], robobj.coordmat[i+1][2])
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
        jointx[i] = robobj.coordmat[i][0]
        jointy[i] = robobj.coordmat[i][1]
        jointz[i] = robobj.coordmat[i][2]
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

    return data
