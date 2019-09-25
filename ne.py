# -*- coding: utf-8 -*-
"""
Created on Sat Sep 21 11:16:32 2019

@author: robot
"""

from sympy import Plane, Point3D
from sympy.abc import x
import numpy as np
import math
import plotly.graph_objects as go
import numpy as np
from  enum import Enum
import plotly
from  math import  cos, sin

import numpy.linalg as lg



import dubins_2Dpath_planning as db


def distance(p1,p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p1[2]) ** 2)



def pnt2DConvert2Pnt3D(p2d_x,p2d_y, mat2Dconvert3D):
    p3D = mat2Dconvert3D.dot(np.array([p2d_x,p2d_y,0],dtype= 'float').T)
    return list (p3D)

def pnt3DConvert2Pnt2D(p3d_x,p3d_y,p3d_z, mat3Dconvert2D):
    print('mat3',mat3Dconvert2D)
    xx = np.array([[p3d_x, p3d_y, p3d_z]], dtype='float').transpose()
    print('xx = ',xx)
    p2D  = mat3Dconvert2D.dot(xx)
    # print('p2D', p2D)
    # exit()
    # raise Exception('XX')
    return list(p2D)


def Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, rad, mat2Dconvert3D,mat3Dconvert2D):

    p2_x, p2_y, p2_z = pnt3DConvert2Pnt2D(goal_x - start_x, goal_y - start_y, goal_z - start_z, mat3Dconvert2D)


    print(p2_x,p2_y,p2_z)
    # raise  Exception('x')

    px, py, pyaw, mode, clen = db.dubins_path_planning(0,0,0,p2_x, p2_y,rad, 1000)

    pnt3D_xLst = []
    pnt3D_yLst = []
    pnt3D_zLst = []

    for i in range(len(px)):
        p3_x,p3_y,p3_z = pnt2DConvert2Pnt3D(px[i],py[i],mat2Dconvert3D)
        pnt3D_xLst.append(p3_x)
        pnt3D_yLst.append(p3_y)
        pnt3D_zLst.append(p3_z)

    pnt3D_xLst = [x + start_x for x in pnt3D_xLst]
    pnt3D_yLst = [y + start_y for y in pnt3D_yLst]
    pnt3D_zLst = [z + start_z for z in pnt3D_zLst]

    print(p2_x)
    print(p2_y)
    print(pyaw[-1])
    p2_x_step = p2_x[0] + 1000*cos(pyaw[-1])
    p2_y_step = p2_y[0] + 1000*sin(pyaw[-1])

    print(p2_x_step,p2_y_step)
    # exit()
    print('mat2Dconvert3D', mat2Dconvert3D)
    p3_x, p3_y, p3_z = pnt2DConvert2Pnt3D(p2_x_step, p2_y_step, mat2Dconvert3D)
    print('mat2Dconvert3D', mat2Dconvert3D)

    p3_x = p3_x + start_x
    p3_y = p3_y + start_y
    p3_z = p3_z + start_z


    bias_x = p3_x - goal_x
    bias_y = p3_y - goal_y
    bias_z = p3_z - goal_z

    a_gama = math.atan(bias_z/2000)

    if bias_y >0:
        a_psi = math.atan(bias_y/bias_x)
    else:
        a_psi = math.atan(bias_y/bias_x) + math.pi


    # py = [y + start_y for y in py]
    return pnt3D_xLst,pnt3D_yLst,pnt3D_zLst,psi,gama, p3_x, p3_y, p3_z
    # pass


start_x = 5000
start_y = 6000
start_z = 2229
psi = np.deg2rad(75)
gama = np.deg2rad(-25)



50000.0
5000.0

dir_x = start_x + 1000 * math.cos(psi)
dir_y = start_y + 1000 * math.sin(psi)
dir_z = start_z + 1000 * math.sin(gama)


goal_x = 6000
goal_y = 8000
goal_z = 3450




start_x = 0.0
start_y = 50000.0
start_z = 5000.0
psi = np.deg2rad(270)
gama = np.deg2rad(-25)



50000.0
5000.0

dir_x = start_x + 1000 * math.cos(psi)
dir_y = start_y + 1000 * math.sin(psi)
dir_z = start_z + 1000 * math.sin(gama)


goal_x = 11392.9607416196
goal_y = 56973.0182393612
goal_z = 4097.85801775604



# psi = np.deg2rad(75)
# gama = np.deg2rad(-25)

vec = [math.cos(psi),math.sin(psi),math.sin(gama)]

import numpy as np

p1 = np.array([start_x, start_y, start_z])
p2 = np.array([dir_x, dir_y, dir_z])
p3 = np.array([goal_x, goal_y, goal_z])

# These two vectors are in the plane
v1 = p3 - p1
v2 = p2 - p1

# the cross product is a vector normal to the plane
cp = np.cross(v1, v2)
a, b, c = cp

# This evaluates a * x3 + b * y3 + c * z3 which equals d
d = np.dot(cp, p3)

print('The equation is {0}x + {1}y + {2}z = {3}'.format(a, b, c, d))



P = Plane(Point3D(start_x, start_y, start_z), Point3D(dir_x, dir_y, dir_z),Point3D(goal_x, goal_y, goal_z))

print(P.equation())
nor_vec = P.normal_vector

print(nor_vec)







def angle(v1, v2, acute):
# v1 is your firsr vector
# v2 is your second vector
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle

# dist = np.linalg.norm(a-b)


ang = angle(vec,[goal_x -start_x, goal_y - start_y, goal_z - goal_z],True)

a3D = np.array([[nor_vec[0], dir_x -start_x, goal_x - start_x],
          [nor_vec[1], dir_y - start_y, goal_y - start_y],
          [nor_vec[2], dir_z - start_z, goal_z- start_z]],dtype = 'float')


print(a3D)

# print()
dist_g = distance([goal_x -start_x, goal_y - start_y, goal_z - start_z],[0,0,0])

# dist_a =
dist_v = distance(nor_vec,[0,0,0])

    # np.linalg.norm(np.array(nor_vec)-np.array([0,0,0]))



a2D = np.array([[0, 1000, dist_g*cos(ang)],
                [0,0,dist_g*sin(ang)],
                [dist_v,0,0]
])
print(a2D)

mat2Dconvert3D = a3D.dot(lg.inv(a2D))
mat3Dconvert2D = a2D.dot(np.linalg.inv(a3D))

# print(T_3vect2)
print('inv 2d')
print(np.linalg.inv(a2D))

print('inv 3d')
print(np.linalg.inv(a3D))



# print(mat3Dconvert2D.dot(np.array([nor_vec],dtype= 'float').transpose()))



# np.array([[0,0,0]])


px,py,pz,psi,gama, p3_x, p3_y, p3_z = Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, math.pi,mat2Dconvert3D = mat2Dconvert3D,mat3Dconvert2D = mat3Dconvert2D)

_scatterLst = []

m_x = []
m_y = []
m_z = []

t_x = []
t_y = []
t_z = []

# for x in range(0,15000,200):
#     for y in range(48000,60000,200):
#         z = (d - a * x - b *y)/c
#         m_x.append(x)
#         m_y.append(y)
#         m_z.append(z)


for x in range(0,15000,200):
    for y in range(48000,60000,200):
        z = (d - a * x - b *y)/c
        m_x.append(x)
        m_y.append(y)
        m_z.append(z)

# for x in range(-3000, 3000, 25):
# for x in range(-10,10):
#     for y in range(-3000, 4500, 25):
#         p3x,p3y,p3z = pnt2DConvert2Pnt3D(0, y,psi,gama)
#         t_x.append(p3x + start_x)
#         t_y.append(p3y + start_y)
#         t_z.append(p3z + start_z)



d_x = []
d_y = []
d_z = []

for i in range(len(px)):
    d_x.append(px[i])
    d_y.append(py[i])
    d_z.append(pz[i])


_scatterLst.append(go.Scatter3d(x=[float(start_x)], y=[float(start_y)], z=[float(start_z)],
                                     mode='markers', marker=dict(size=5), name='A'))

_scatterLst.append(go.Scatter3d(x=[float(start_x),dir_x], y=[float(start_y),dir_y], z=[float(start_z),dir_z],
                                     mode='lines',line=dict(width=5), name='L'))


_scatterLst.append(go.Scatter3d(x=[float(goal_x),p3_x], y=[float(goal_y),p3_y], z=[float(goal_z),p3_z],
                                     mode='lines',line=dict(width=5), name='G'))

_scatterLst.append(go.Scatter3d(x=[float(goal_x)], y=[float(goal_y)], z=[float(goal_z)],
                                     mode='markers', marker=dict(size=5), name='B'))
_scatterLst.append(go.Mesh3d(x=m_x, y=m_y, z=m_z, opacity=0.50))

# _scatterLst.append(go.Mesh3d(x=t_x, y=t_y, z=t_z, opacity=0.50))


_scatterLst.append(go.Scatter3d(x=d_x, y=d_y, z=d_z, mode='lines', line=dict(width=5), name='dUBINS'))
# self._scatterLst.append(go.Scatter3d(x=p_x, y=p_y, z=p_z, mode='lines', line=dict(width=3), name='path'))

fig = go.Figure(data=_scatterLst)
plotly.offline.plot(fig, filename='test_ins_1_dubins.html')








