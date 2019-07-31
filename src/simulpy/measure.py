import numpy as np
from itertools import product
from simulpy import kin
import plotly.graph_objs as go


def work_space(robobj, limit1, limit2):

    x_range = np.linspace(limit1[0], limit2[0])
    y_range = np.linspace(limit1[1], limit2[1])
    z_range = np.linspace(limit1[2], limit2[2])

    allpts = product(x_range, y_range, z_range)
    print(allpts)
    i = 0

    mat_x = np.zeros(2**robobj.jointno)
    mat_y = np.zeros(2**robobj.jointno)
    mat_z = np.zeros(2**robobj.jointno)

    while i < 2**robobj.jointno:
        m = kin.fwd(robobj, cp[i])
        mat_x[i] = m[robobj.jointno][0]
        mat_y[i] = m[robobj.jointno][1]
        mat_z[i] = m[robobj.jointno][2]
        i = i + 1

        vol = go.Mesh3d(x=mat_x,
                   y=mat_y,
                   z=mat_z,
                   opacity=1,
                   color='rgba(244,22,100,0.6)'
                  )
    return vol
