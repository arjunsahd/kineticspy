import numpy as np
import plotly.graph_objs as go
from simulpy.robobj import robot

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

    data = [vol]

    return data


#needed fixing future project
def obstacle(l, b, h, pos):

    x_neg = pos[0] - l/2
    x_pos = pos[0] + l/2

    y_neg = pos[1] - b/2
    y_pos = pos[1] + b/2

    z_neg = pos[2] - h/2
    z_pos = pos[2] + h/2

    offset = 0.01

    x_arr1 = np.zeros(4)
    y_arr1 = np.zeros(4)
    z_arr1 = np.zeros(4)

    [x_arr1[0], y_arr1[0], z_arr1[0]] = [x_pos + offset, y_pos, z_pos]
    [x_arr1[1], y_arr1[1], z_arr1[1]] = [x_pos, y_pos, z_neg]
    [x_arr1[2], y_arr1[2], z_arr1[2]] = [x_pos, y_neg, z_neg]
    [x_arr1[3], y_arr1[3], z_arr1[3]] = [x_pos + offset, y_neg, z_pos]

    x_arr2 = np.zeros(4)
    y_arr2 = np.zeros(4)
    z_arr2 = np.zeros(4)

    [x_arr2[0], y_arr2[0], z_arr2[0]] = [x_pos, y_pos + offset, z_pos]
    [x_arr2[1], y_arr2[1], z_arr2[1]] = [x_pos, y_pos, z_neg]
    [x_arr2[2], y_arr2[2], z_arr2[2]] = [x_neg, y_pos, z_neg]
    [x_arr2[3], y_arr2[3], z_arr2[3]] = [x_neg, y_pos + offset, z_pos]

    x_arr3 = np.zeros(4)
    y_arr3 = np.zeros(4)
    z_arr3 = np.zeros(4)

    [x_arr3[0], y_arr3[0], z_arr3[0]] = [x_neg - offset, y_pos, z_pos]
    [x_arr3[1], y_arr3[1], z_arr3[1]] = [x_neg - offset, y_neg, z_pos]
    [x_arr3[2], y_arr3[2], z_arr3[2]] = [x_neg, y_neg, z_neg]
    [x_arr3[3], y_arr3[3], z_arr3[3]] = [x_neg, y_pos, z_neg]

    x_arr4 = np.zeros(4)
    y_arr4 = np.zeros(4)
    z_arr4 = np.zeros(4)

    [x_arr4[0], y_arr4[0], z_arr4[0]] = [x_pos, y_neg - offset, z_pos]
    [x_arr4[1], y_arr4[1], z_arr4[1]] = [x_pos, y_neg, z_neg]
    [x_arr4[2], y_arr4[2], z_arr4[2]] = [x_neg, y_neg, z_neg]
    [x_arr4[3], y_arr4[3], z_arr4[3]] = [x_neg, y_neg - offset, z_pos]

    x_arr5 = np.zeros(4)
    y_arr5 = np.zeros(4)
    z_arr5 = np.zeros(4)

    [x_arr5[0], y_arr5[0], z_arr5[0]] = [x_pos, y_pos, z_pos]
    [x_arr5[1], y_arr5[1], z_arr5[1]] = [x_pos, y_neg, z_pos]
    [x_arr5[2], y_arr5[2], z_arr5[2]] = [x_neg, y_pos, z_pos]
    [x_arr5[3], y_arr5[3], z_arr5[3]] = [x_neg, y_neg, z_pos]

    x_arr6 = np.zeros(4)
    y_arr6 = np.zeros(4)
    z_arr6 = np.zeros(4)

    [x_arr6[0], y_arr6[0], z_arr6[0]] = [x_pos, y_pos, z_neg]
    [x_arr6[1], y_arr6[1], z_arr6[1]] = [x_pos, y_neg, z_neg]
    [x_arr6[2], y_arr6[2], z_arr6[2]] = [x_neg, y_pos, z_neg]
    [x_arr6[3], y_arr6[3], z_arr6[3]] = [x_neg, y_neg, z_neg]


    data1 = go.Mesh3d(x= x_arr1, y = y_arr1, z= z_arr1,
                      color ='red',
                      opacity = 0.5)
    data2 = go.Mesh3d(x= x_arr2, y = y_arr2, z= z_arr2,
                      color ='red',
                      opacity = 0.5)
    data3 = go.Mesh3d(x= x_arr3, y = y_arr3, z= z_arr3,
                      color ='red',
                      opacity = 0.5)
    data4 = go.Mesh3d(x= x_arr4, y = y_arr4, z= z_arr4,
                      color ='red',
                      opacity = 0.5)
    data5 = go.Mesh3d(x= x_arr5, y = y_arr5, z= z_arr5,
                      color ='red',
                      opacity = 0.5)
    data6 = go.Mesh3d(x= x_arr6, y = y_arr6, z= z_arr6,
                      color ='red',
                      opacity = 0.5)

    data = [data1, data2, data3, data4, data5, data6]

    return data
