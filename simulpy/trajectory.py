import numpy as np
from simulpy import kin
import copy
import plotly
import plotly.graph_objs as go


def calctrajectorymat(thetamat, robobj, nopoints):

    i = 0
    trajectorymat = np.zeros((nopoints+2, robobj.jointno + 1, 3))

    while i < nopoints + 2:
        kin.fwd(robobj, thetamat[i])
        trajectorymat[i] = robobj.coordmat
        i = i + 1

    return trajectorymat


def calcont(robobj, thetainit, thetafinal, nopoints):

    thetamat = np.zeros((nopoints + 2, robobj.jointno))
    thetamat[0] = thetainit

    i = 1
    while i < nopoints + 2:
        j = 0
        while j < robobj.jointno:
            thetamat[i][j] = thetainit[j] + i*thetafinal[j]/(nopoints+1)
            j = j + 1
        i = i + 1

    return calctrajectorymat(thetamat, robobj, nopoints)


def calcdisc(robobj, thetainit, thetafinal, nopoints):

    updnopoints = nopoints*robobj.jointno + robobj.jointno + 1

    thetamat = np.zeros((updnopoints, robobj.jointno))
    thetastep = np.zeros((robobj.jointno + 1, robobj.jointno))
    thetastep[0] = thetainit

    i = 1
    while i < robobj.jointno + 1:
        thetastep[i] = thetastep[i-1]
        thetastep[i][i-1] = thetafinal[i-1]
        i = i + 1

    thetamat[0] = thetastep[0]
    i = 0
    j = 1
    initj = 0
    while i < robobj.jointno:
        while j < initj + nopoints + 1:
            thetamat[j] = thetastep[i] + (j-initj)*(thetastep[i+1] - thetastep[i])/(nopoints + 1)
            j = j + 1
        initj = copy.deepcopy(j)
        i = i + 1

    return calctrajectorymat(thetamat, robobj, updnopoints)


def plotcont(robobj, trajectorymat, nopoints, opt):

    plotly.offline.init_notebook_mode(connected=True)

    if opt == 1:

        i = 0
        initj = 0
        x = np.linspace(trajectorymat[0][i][0], trajectorymat[0][i+1][0])
        lenx = len(x)
        x_arr = np.zeros(3*lenx)
        y_arr = np.zeros(3*lenx)
        z_arr = np.zeros(3*lenx)

        while i < robobj.jointno:
            x = np.linspace(trajectorymat[0][i][0], trajectorymat[0][i+1][0])
            y = np.linspace(trajectorymat[0][i][1], trajectorymat[0][i+1][1])
            z = np.linspace(trajectorymat[0][i][2], trajectorymat[0][i+1][2])
            j = copy.deepcopy(initj)

            while j < lenx + initj:
                x_arr[j] = x[j-initj]
                y_arr[j] = y[j-initj]
                z_arr[j] = z[j-initj]
                j = j + 1

            i = i + 1
            initj = copy.deepcopy(j)

        armvec1 = go.Scatter3d(
            x=x_arr, y=y_arr, z=z_arr,
            marker=dict(
                size=2,
            ),
            line=dict(
                color='#8c564b',
                width=1
            )
        )

        i = 0

        jointx = np.zeros(robobj.jointno + 1)
        jointy = np.zeros(robobj.jointno + 1)
        jointz = np.zeros(robobj.jointno + 1)

        while i < robobj.jointno + 1:
            jointx[i] = trajectorymat[0][i][0]
            jointy[i] = trajectorymat[0][i][1]
            jointz[i] = trajectorymat[0][i][2]
            i = i + 1

        joint1 = go.Scatter3d(
            x=jointx, y=jointy, z=jointz,
            marker=dict(
                size=4,
            ),
            line=dict(
                color='#d62728',
                width=2
            )
        )

        i = 0
        initj = 0
        x2 = np.linspace(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i+1][0])
        lenx = len(x2)
        x_arr2 = np.zeros(3*lenx)
        y_arr2 = np.zeros(3*lenx)
        z_arr2 = np.zeros(3*lenx)

        while i < robobj.jointno:
            x2 = np.linspace(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i+1][0])
            y2 = np.linspace(trajectorymat[nopoints + 1][i][1], trajectorymat[nopoints + 1][i+1][1])
            z2 = np.linspace(trajectorymat[nopoints + 1][i][2], trajectorymat[nopoints + 1][i+1][2])
            j = copy.deepcopy(initj)

            while j < lenx + initj:
                x_arr2[j] = x2[j-initj]
                y_arr2[j] = y2[j-initj]
                z_arr2[j] = z2[j-initj]
                j = j + 1

            i = i + 1
            initj = copy.deepcopy(j)

        armvec2 = go.Scatter3d(
            x=x_arr2, y=y_arr2, z=z_arr2,
            marker=dict(
                size=2,
            ),
            line=dict(
                color='#1f77b4',
                width=1
            )
        )

        i = 0

        jointx2 = np.zeros(robobj.jointno + 1)
        jointy2 = np.zeros(robobj.jointno + 1)
        jointz2 = np.zeros(robobj.jointno + 1)

        while i < robobj.jointno + 1:
            jointx2[i] = trajectorymat[nopoints + 1][i][0]
            jointy2[i] = trajectorymat[nopoints + 1][i][1]
            jointz2[i] = trajectorymat[nopoints + 1][i][2]
            i = i + 1

        joint2 = go.Scatter3d(
            x=jointx2, y=jointy2, z=jointz2,
            marker=dict(
                size=5,
            ),
            line=dict(
                color='#ff7f0e',
                width=2
            )
        )

        x_array = np.zeros(nopoints + 2)
        y_array = np.zeros(nopoints + 2)
        z_array = np.zeros(nopoints + 2)

        i = 0
        while i < nopoints + 2:
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

        data = [armvec1, joint1, armvec2, joint2, trac]
        fig = go.Figure(data=data, layout={})
        plotly.offline.iplot(fig)

    else:
        x_array = np.zeros(nopoints + 2)
        y_array = np.zeros(nopoints + 2)
        z_array = np.zeros(nopoints + 2)

        i = 0
        while i < nopoints + 2:
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

        data = [trac]
        fig = go.Figure(data=data, layout={})
        plotly.offline.iplot(fig)


def plotdisc(robobj, trajectorymat, nopoints, opt):

    plotly.offline.init_notebook_mode(connected=True)
    nopoints = copy.deepcopy(nopoints*robobj.jointno + robobj.jointno - 1)

    if opt == 1:

        i = 0
        initj = 0

        x = np.linspace(trajectorymat[3][i][0], trajectorymat[3][i+1][0])
        lenx = len(x)
        x_arr = np.zeros(3*lenx)
        y_arr = np.zeros(3*lenx)
        z_arr = np.zeros(3*lenx)

        while i < robobj.jointno:
            x = np.linspace(trajectorymat[3][i][0], trajectorymat[3][i+1][0])
            y = np.linspace(trajectorymat[3][i][1], trajectorymat[3][i+1][1])
            z = np.linspace(trajectorymat[3][i][2], trajectorymat[3][i+1][2])
            j = copy.deepcopy(initj)

            while j < lenx + initj:
                x_arr[j] = x[j-initj]
                y_arr[j] = y[j-initj]
                z_arr[j] = z[j-initj]
                j = j + 1

            i = i + 1
            initj = copy.deepcopy(j)

        armvec1 = go.Scatter3d(
            x=x_arr, y=y_arr, z=z_arr,
            marker=dict(
                size=2,
            ),
            line=dict(
                color='#8c564b',
                width=1
            )
        )

        i = 0

        jointx = np.zeros(robobj.jointno + 1)
        jointy = np.zeros(robobj.jointno + 1)
        jointz = np.zeros(robobj.jointno + 1)

        while i < robobj.jointno + 1:
            jointx[i] = trajectorymat[3][i][0]
            jointy[i] = trajectorymat[3][i][1]
            jointz[i] = trajectorymat[3][i][2]
            i = i + 1

        joint1 = go.Scatter3d(
            x=jointx, y=jointy, z=jointz,
            marker=dict(
                size=4,
            ),
            line=dict(
                color='#d62728',
                width=2
            )
        )

        i = 0
        initj = 0

        x2 = np.linspace(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i+1][0])
        lenx = len(x2)
        x_arr2 = np.zeros(3*lenx)
        y_arr2 = np.zeros(3*lenx)
        z_arr2 = np.zeros(3*lenx)

        while i < robobj.jointno:
            x2 = np.linspace(trajectorymat[nopoints + 1][i][0], trajectorymat[nopoints + 1][i+1][0])
            y2 = np.linspace(trajectorymat[nopoints + 1][i][1], trajectorymat[nopoints + 1][i+1][1])
            z2 = np.linspace(trajectorymat[nopoints + 1][i][2], trajectorymat[nopoints + 1][i+1][2])

            j = copy.deepcopy(initj)
            while j < lenx + initj:
                x_arr2[j] = x2[j-initj]
                y_arr2[j] = y2[j-initj]
                z_arr2[j] = z2[j-initj]
                j = j + 1

            i = i + 1
            initj = copy.deepcopy(j)

        armvec2 = go.Scatter3d(
            x=x_arr2, y=y_arr2, z=z_arr2,
            marker=dict(
                size=2,
            ),
            line=dict(
                color='#1f77b4',
                width=1
            )
        )

        i = 0

        jointx2 = np.zeros(robobj.jointno + 1)
        jointy2 = np.zeros(robobj.jointno + 1)
        jointz2 = np.zeros(robobj.jointno + 1)

        while i < robobj.jointno + 1:
            jointx2[i] = trajectorymat[nopoints + 1][i][0]
            jointy2[i] = trajectorymat[nopoints + 1][i][1]
            jointz2[i] = trajectorymat[nopoints + 1][i][2]
            i = i + 1

        joint2 = go.Scatter3d(
            x=jointx2, y=jointy2, z=jointz2,
            marker=dict(
                size=4,
            ),
            line=dict(
                color='#ff7f0e',
                width=2
            )
        )

        x_array = np.zeros(nopoints + 2)
        y_array = np.zeros(nopoints + 2)
        z_array = np.zeros(nopoints + 2)

        i = 0
        while i < nopoints + 2:
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

        data = [armvec1, joint1, armvec2, joint2, trac]
        fig = go.Figure(data=data, layout={})
        plotly.offline.iplot(fig)

    else:
        x_array = np.zeros(nopoints + 2)
        y_array = np.zeros(nopoints + 2)
        z_array = np.zeros(nopoints + 2)

        i = 0
        while i < nopoints + 2:
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

        data = [trac]
        fig = go.Figure(data=data, layout={})
        plotly.offline.iplot(fig)
