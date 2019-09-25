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
from enum import Enum
import plotly
from math import cos, sin

import numpy.linalg as lg

import dubins_2Dpath_planning as db


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])** 2 + (p1[1] - p2[1])** 2 + (p1[2] - p2[2])** 2)


def pnt2DConvert2Pnt3D(p2d_x, p2d_y, mat2Dconvert3D):
    xx = np.array([[p2d_x, p2d_y, 0]], dtype='float').transpose()
    p3D = mat2Dconvert3D.dot(xx)
    p3DLst = []
    for x in p3D:
        p3DLst.append(float(x))
    return p3DLst


def pnt3DConvert2Pnt2D(p3d_x, p3d_y, p3d_z, mat3Dconvert2D):
    # print('mat3', mat3Dconvert2D)
    xx = np.array([[p3d_x, p3d_y, p3d_z]], dtype='float').transpose()
    # print('xx = ', xx)
    p2D = mat3Dconvert2D.dot(xx)
    # print('p2D', p2D)
    # exit()
    # raise Exception('XX')
    p2DLst = []
    for x in p2D:
        p2DLst.append(float(x))
    return p2DLst


def in_gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, rad, mat2Dconvert3D, mat3Dconvert2D):


    # print(goal_x,goal_y,goal_z)

    p2_x, p2_y, p2_z = pnt3DConvert2Pnt2D(goal_x - start_x, goal_y - start_y, goal_z - start_z, mat3Dconvert2D)

    print('XXX PPP',float(p2_x), p2_y, p2_z)

    # raise  Exception('x')

    # exit()
    # wtf_x, wtf_y, wtf_z = pnt2DConvert2Pnt3D(p2_x, p2_x, mat2Dconvert3D)
    #
    # print(wtf_x + start_x,wtf_y + start_y,wtf_z + start_z)


    # print('rad = ',rad)

    # print('需要改半径')
    print(rad)
    px, py, pyaw, mode, clen = db.dubins_path_planning(0, 0, 0, p2_x, p2_y, rad, 200)


    dis_d  = 0
    for i in range(len(px) -1):
        dis_d  += math.sqrt((px[i] - px[i + 1])**2 +  (py[i] - py[i + 1])**2)

    print(dis_d)
    # print(np.linalg.norm([px,py]))
    print(clen*200)
    # exit()
    # print('inside_dis',dis_d)
    # print(mode)
    # print(dis_d)

    pnt3D_xLst = []
    pnt3D_yLst = []
    pnt3D_zLst = []

    for i in range(len(px)):
        p3_x, p3_y, p3_z = pnt2DConvert2Pnt3D(px[i], py[i], mat2Dconvert3D)
        pnt3D_xLst.append(p3_x)
        pnt3D_yLst.append(p3_y)
        pnt3D_zLst.append(p3_z)

    pnt3D_xLst = [x + start_x for x in pnt3D_xLst]
    pnt3D_yLst = [y + start_y for y in pnt3D_yLst]
    pnt3D_zLst = [z + start_z for z in pnt3D_zLst]

    # print(pyaw[-1])

    p2_x_step = 2000*cos(pyaw[-1]) + p2_x
    p2_y_step = 2000*sin(pyaw[-1]) + p2_y

    # print(p2_x_step**2+ p2_y_step**2)
    p3_x, p3_y, p3_z = pnt2DConvert2Pnt3D(p2_x_step, p2_y_step, mat2Dconvert3D)


    # print('bfp3_x',p3_x)
    # print('bfp3_y',p3_y)
    # print('bfp3_z',p3_z)


    p3_x = p3_x + start_x
    p3_y = p3_y + start_y
    p3_z = p3_z + start_z

    # print('p3_x',p3_x)
    # print('p3_y',p3_y)
    # print('p3_z',p3_z)

    bias_x = p3_x - goal_x
    bias_y = p3_y - goal_y
    bias_z = p3_z - goal_z



    # print('b_x',bias_x)
    # print('b_y',bias_y)
    # print('b_z',bias_z)

    # print('dis = ',distance([bias_x,bias_y,bias_z],[0,0,0]))


    # bias_x = pnt3D_xLst[-1] - pnt3D_xLst[-2]
    # bias_y = pnt3D_yLst[-1] - pnt3D_yLst[-2]
    # bias_z = pnt3D_zLst[-1] - pnt3D_zLst[-2]
    #
    # print('b_x', bias_x)
    # print('b_y', bias_y)
    # print('b_z', bias_z)
    #
    # print('dis = ',distance([bias_x,bias_y,bias_z],[0,0,0]))

    a_gama = math.atan(bias_z / 2000)
    # if a_gama >= 0:
    #     print('a_gama >0')
    #     a_gama = a_gama
    # else:
    #     print('a_gama <0')
    #     a_gama = math.pi + a_gama

    if bias_x > 0 and bias_y > 0 :
        # print('case1')
        a_psi = math.atan(bias_y / bias_x)
    elif bias_x < 0 and bias_y <=0:
        # print('case2')
        a_psi = math.atan(bias_y / bias_x) + math.pi
    elif bias_x > 0 and bias_y <=0:
        # print('case3')
        a_psi = math.atan(bias_y / bias_x)
    else:
        # print('case4')
        a_psi = math.atan(bias_y / bias_x) + math.pi
    return pnt3D_xLst, pnt3D_yLst, pnt3D_zLst, dis_d, a_psi, a_gama, p3_x, p3_y, p3_z


def angle(v1, v2, acute):
    # v1 is your firsr vector
    # v2 is your second vector
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
    if (acute == True):
        return angle
    else:
        return 2 * np.pi - angle


def Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, psi, gama, rad,drawB = False):
    # start_dir_z = math.sqrt(1 - start_dir_x ** 2 - start_dir_y ** 2)
    # start_x = 0
    # start_y = 0
    # start_z = 0
    #
    # goal_x = 0
    # goal_y = 0
    # goal_z = 100
    # psi = math.pi/4
    # gama = math.pi/4
    vec = [math.cos(psi)* math.cos(gama),math.sin(psi) * math.cos(gama),math.sin(gama)]

    # print('vec_dis',distance(vec,[0,0,0]))
    # print('vec_dis2',np.linalg.norm(np.array(vec)-np.array([0,0,0])))

    dir_x = start_x + 1000 * math.cos(psi) * math.cos(gama)
    dir_y = start_y + 1000 * math.sin(psi) * math.cos(gama)
    dir_z = start_z + 1000 * math.sin(gama)


    # print('dir_dis',distance([dir_x,dir_y,dir_z],[start_x, start_y, start_z]))
    if  -3.15/2 < gama and gama <3.15/2:
        pass
    else:
        print(gama)
        raise Exception ('gama')


    P = Plane(Point3D(start_x, start_y, start_z), Point3D(dir_x, dir_y, dir_z), Point3D(goal_x, goal_y, goal_z))



    nor_vec = P.normal_vector

    nor_vec = [float(x)  for  x in nor_vec]
    # print(nor_vec)

    ang = angle(vec, [goal_x - start_x, goal_y - start_y, goal_z - start_z], True)

    a3D = np.mat([[nor_vec[0], dir_x - start_x, goal_x - start_x],
                    [nor_vec[1], dir_y - start_y, goal_y - start_y],
                    [nor_vec[2], dir_z - start_z, goal_z - start_z]], dtype='float')


    # print('a3D',a3D)
    # print()
    dist_g = distance([goal_x - start_x, goal_y - start_y, goal_z - start_z], [0, 0, 0])
    # print('dist_g',dist_g)
    # print('dir_dis', distance([goal_x, goal_y, goal_z], [start_x, start_y, start_z]))
    # dist_a =
    dist_v = distance(nor_vec, [0, 0, 0])

    # print('dist_v',dist_v)
    a2D = np.mat([[0, 1000, dist_g * cos(ang)],
                    [0, 0, dist_g * sin(ang)],
                    [dist_v, 0, 0]])

    # print('a2D',a2D)

    mat2Dconvert3D = a3D.dot(lg.inv(a2D))

    a3Dni = np.linalg.inv(a3D)
    mat3Dconvert2D = np.dot(a2D,a3Dni)


    mat2Dconvert3D = np.linalg.inv(mat3Dconvert2D)

    # print(mat3Dconvert2D)

    # print(mat2Dconvert3D)

    # print(mat2Dconvert3D.dot(mat3Dconvert2D))
    # exit()

    # print(mat3Dconvert2D)


    # mat3Dconvert2D = np.multiply(a2D,a3Dni)
    # print(mat3Dconvert2D)

    # print('a3D ni', np.linalg.inv(a3D))
    # print('2D*3D')

    # print(mat3Dconvert2D)
    # print('a3D inv',np.linalg.inv(a3D))

    # exit()

    # print()
    # print('xxx',mat3Dconvert2D.dot(mat2Dconvert3D))

    # exit()

    # print(mat2Dconvert3D)
    # print(mat3Dconvert2D)

    px, py, pz, dis_d, a_psi, a_gama, p3_x, p3_y, p3_z = in_gdubins3D(
                                   start_x =  start_x,  start_y = start_y,  start_z = start_z,
                                    goal_x= goal_x, goal_y = goal_y, goal_z = goal_z, rad = rad,
                                            mat2Dconvert3D=mat2Dconvert3D,
                                            mat3Dconvert2D=mat3Dconvert2D)
    if drawB:
        _scatterLst = []

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

        m_x = []
        m_y = []
        m_z = []

        t_x = []
        t_y = []
        t_z = []

        for x in range(0,15000,200):
            for y in range(48000,60000,200):
                z = (d - a * x - b *y)/c
                m_x.append(x)
                m_y.append(y)
                m_z.append(z)

        # import numpy as np

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

        for x in range(0, 15000, 200):
            for y in range(48000, 60000, 200):
                z = (d - a * x - b * y) / c
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

        _scatterLst.append(go.Scatter3d(x=[float(start_x)], y=[float(start_y)], z=[float(start_z)],
                                        mode='markers', marker=dict(size=5), name='起始点'))
        _scatterLst.append(go.Scatter3d(x=[float(goal_x)], y=[float(goal_y)], z=[float(goal_z)],
                                        mode='markers', marker=dict(size=5), name='终止点'))


        _scatterLst.append(go.Scatter3d(x=[float(start_x), dir_x], y=[float(start_y), dir_y], z=[float(start_z), dir_z],
                                        mode='lines', line=dict(width=5), name='起始点朝向'))


        dir_x = goal_x - 500 * math.cos(a_psi)*cos(a_gama)
        dir_y = goal_y - 500 * math.sin(a_psi)*cos(a_gama)
        dir_z = goal_z - 500 * math.sin(a_gama)

        print(dir_x)
        print(dir_y)
        print(dir_z)

        print(p3_x)
        print(p3_y)
        print(p3_z)


        print('dis = ',distance([dir_x,dir_y,dir_z],[p3_x,p3_y,p3_z]))

        d_x = []
        d_y = []
        d_z = []

        for i in range(len(px)):
            d_x.append(px[i])
            d_y.append(py[i])
            d_z.append(pz[i])




        # _scatterLst.append(go.Scatter3d(x=[float(p3_x)], y=[float(p3_y)], z=[float( p3_z)],
        #                                 mode='markers', marker=dict(size=5), name='p'))


        _scatterLst.append(go.Scatter3d(x=[float(goal_x), p3_x], y=[float(goal_y), p3_y], z=[float(goal_z), p3_z],
                                        mode='lines', line=dict(width=5), name='目标点朝向'))


        _scatterLst.append(go.Mesh3d(x=m_x, y=m_y, z=m_z, opacity=0.50))

        # _scatterLst.append(go.Mesh3d(x=t_x, y=t_y, z=t_z, opacity=0.50))

        _scatterLst.append(go.Scatter3d(x=d_x, y=d_y, z=d_z, mode='lines', line=dict(width=5), name='dubins运动轨迹'))
        # self._scatterLst.append(go.Scatter3d(x=p_x, y=p_y, z=p_z, mode='lines', line=dict(width=3), name='path'))

        # _scatterLst.append(go.Scatter3d(x=[dir_x, p3_x], y=[dir_y, p3_y], z=[dir_z, p3_z],
        #                                 mode='lines', line=dict(width=5), name='G_dir'))

        fig = go.Figure(data=_scatterLst)

        plotly.offline.plot(fig, filename='test_ins_7_dubins.html')


    # print(start_x)
    # print(start_y)
    # print(start_z)
    #
    # print(goal_x)
    # print(goal_y)
    # print(goal_z)


    return px, py, pz, dis_d, a_psi, a_gama, p3_x, p3_y, p3_z


if __name__ == '__main__':


    start_x = 0.0
    start_y = 50000.0
    start_z = 444.0
    psi = np.deg2rad(80)
    gama = np.deg2rad(45)

    goal_x = 11392.9607416196
    goal_y = 56973.0182393612
    goal_z = 6000.85801775604

    Gdubins3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, psi, gama, math.pi, drawB = True)
    pass


    # _scatterLst = []
    #
    # m_x = []
    # m_y = []
    # m_z = []
    #
    # t_x = []
    # t_y = []
    # t_z = []
    #
    # for x in range(4500, 6500, 25):
    #     for y in range(5500, 8500, 25):
    #         z = (d - a * x - b * y) / c
    #         m_x.append(x)
    #         m_y.append(y)
    #         m_z.append(z)
    #
    # # for x in range(-3000, 3000, 25):
    # # for x in range(-10,10):
    # #     for y in range(-3000, 4500, 25):
    # #         p3x,p3y,p3z = pnt2DConvert2Pnt3D(0, y,psi,gama)
    # #         t_x.append(p3x + start_x)
    # #         t_y.append(p3y + start_y)
    # #         t_z.append(p3z + start_z)
    #
    # d_x = []
    # d_y = []
    # d_z = []
    #
    # for i in range(len(px)):
    #     d_x.append(px[i])
    #     d_y.append(py[i])
    #     d_z.append(pz[i])
    #
    # _scatterLst.append(go.Scatter3d(x=[float(start_x)], y=[float(start_y)], z=[float(start_z)],
    #                                 mode='markers', marker=dict(size=15), name='A'))
    #
    # _scatterLst.append(go.Scatter3d(x=[float(start_x), dir_x], y=[float(start_y), dir_y], z=[float(start_z), dir_z],
    #                                 mode='lines', line=dict(width=5), name='L'))
    #
    # _scatterLst.append(go.Scatter3d(x=[float(goal_x)], y=[float(goal_y)], z=[float(goal_z)],
    #                                 mode='markers', marker=dict(size=15), name='B'))
    # _scatterLst.append(go.Mesh3d(x=m_x, y=m_y, z=m_z, opacity=0.50))
    #
    # # _scatterLst.append(go.Mesh3d(x=t_x, y=t_y, z=t_z, opacity=0.50))
    #
    # _scatterLst.append(go.Scatter3d(x=d_x, y=d_y, z=d_z, mode='lines', line=dict(width=5), name='dUBINS'))
    #
    #
