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

    px, py, pyaw, mode, dis_d = db.dubins_path_planning(0,0,0,p2_x, p2_y,rad, 200)

    print(mode)
    print(dis_d)


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

    p2_x_step = p2_x + cos(pyaw)
    p2_y_step = p2_y + sin(pyaw)
    p3_x,p3_y,p3_z = pnt2DConvert2Pnt3D(p2_x_step,p2_x_step,mat2Dconvert3D)

    goal_dir_x = p3_x - goal_x
    goal_dir_y = p3_y - goal_y
    goal_dir_z = p3_z - goal_z

    # py = [y + start_y for y in py]
    return pnt3D_xLst,pnt3D_yLst,pnt3D_zLst,dis_d,(goal_dir_x,goal_dir_y,goal_dir_z)


def angle(v1, v2, acute):
# v1 is your firsr vector
# v2 is your second vector
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle


def Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, start_dir_x, start_dir_y,rad):
    start_dir_z = math.sqrt(1- start_dir_x **2 - start_dir_y **2)

    vec = [start_dir_x, start_dir_y, start_dir_z]
    dir_x = start_x + start_dir_x
    dir_y = start_y + start_dir_y
    dir_z = start_z + start_dir_z

    P = Plane(Point3D(start_x, start_y, start_z), Point3D(dir_x, dir_y, dir_z), Point3D(goal_x, goal_y, goal_z))

    nor_vec = P.normal_vector

    ang = angle(vec, [goal_x - start_x, goal_y - start_y, goal_z - start_z], True)

    a3D = np.array([[nor_vec[0], dir_x - start_x, goal_x - start_x],
                    [nor_vec[1], dir_y - start_y, goal_y - start_y],
                    [nor_vec[2], dir_z - start_z, goal_z - start_z]], dtype='float')

    # print(a3D)
    # print()
    dist_g = distance([goal_x - start_x, goal_y - start_y, goal_z - goal_z], [0, 0, 0])

    # dist_a =
    dist_v = distance(nor_vec, [0, 0, 0])

    a2D = np.array([[0, 1000, dist_g * cos(ang)],
                    [0, 0, dist_g * sin(ang)],
                    [dist_v, 0, 0]
                    ])
    # print(a2D)

    mat2Dconvert3D = a3D.dot(lg.inv(a2D))
    mat3Dconvert2D = a2D.dot(np.linalg.inv(a3D))

    px, py, pz,dis_d,goal_dir = Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, math.pi, mat2Dconvert3D=mat2Dconvert3D,
                           mat3Dconvert2D=mat3Dconvert2D)

    return px, py, pz, dis_d, goal_dir








