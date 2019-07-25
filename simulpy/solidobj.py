import numpy as np
import plotly.graph_objs as go


def volcov(robobj, trajectorymat):

    l = len(trajectorymat)

    arr = np.zeros((3, (robobj.jointno - 1)*l + 1))

    arr[0][0] = trajectorymat[0][1][0]
    arr[1][0] = trajectorymat[0][1][1]
    arr[2][0] = trajectorymat[0][1][2]
    i = 0
    m = 1

    while i < l:

        k = 2
        while k < robobj.jointno + 1:
            arr[0][m] = trajectorymat[i][k][0]
            arr[1][m] = trajectorymat[i][k][1]
            arr[2][m] = trajectorymat[i][k][2]
            k = k + 1
            m = m + 1
        i = i + 1

    vol = go.Mesh3d(x=arr[0],
                   y=arr[1],
                   z=arr[2],
                   opacity=0.8,
                   color='rgba(244,22,100,0.6)'
                  )
    return vol
