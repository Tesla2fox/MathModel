#    This file is part of EAP.
#
#    EAP is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as
#    published by the Free Software Foundation, either version 3 of
#    the License, or (at your option) any later version.
#
#    EAP is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with EAP. If not, see <http://www.gnu.org/licenses/>.

import random
import array

import numpy

from deap import base
from deap import benchmarks
from deap import creator
from deap import tools

from heapq import heappush, heappop

import plotly.graph_objects as go
import numpy as np
from  enum import Enum
import plotly

import readcfg as rd
import networkx as nx
from collections import  namedtuple
import math
import sys
import  readcfg as rd


# import dubins_3Dpath_planning as db

import  Gdubins3D as gdb

ER = namedtuple('ER', ['e_v', 'e_h'])

STA = namedtuple('STA',['pos','er'])

STADir = namedtuple('STADir',['dir_pos','er'])

Point3D  = namedtuple('Point3D',['x','y','z'])

DirPoint3D = namedtuple('DirPoint3D',['x','y','z','psi','gama'])


ED = namedtuple('ED',['sInd','tInd'])

# ANode = namedtuple('ANode',['ind','er'])

DIR = namedtuple('DIR',['psi','gama'])
ANode = namedtuple('ANode',['ind','er','psi','gama'])

class CorType(Enum):
    Vertical = 1
    Horizontal = 2
    Start = 3
    Goal = 4



# numpy.random.seed(1)

class Pro_Solver(object):
    def __init__(self, pntLst , typeLst, lst_x, lst_y, lst_z, pnt_A, pnt_B , a1 ,a2 ,b1 ,b2 ,th ):

        self._pntLst = pntLst
        self._pntNum = len(pntLst)
        self._pntBind = len(pntLst) - 1
        self._typeLst = typeLst
        self._pnt_A = pnt_A
        self._pnt_B = pnt_B
        self._a1 = a1
        self._a2 = a2
        self._b1 = b1
        self._b2 = b2
        self._th = th

        # print(self._b1)
        # raise Exception('xx')
        self._lst_x = lst_x
        self._lst_y = lst_y
        self._lst_z = lst_z

        self._min_x = min(self._lst_x)
        self._min_y = min(self._lst_y)
        self._min_z = min(self._lst_z)

        self._max_x = max(self._lst_x)
        self._max_y = max(self._lst_y)
        self._max_z = max(self._lst_z)
        # self.calDisMat()

        self._init_sta = STA(pos = self._pnt_A, er = ER(0,0))

        self._path = []
        self._pathLst = []

        self._Rmin = 200

        self._path = [0, 503, 200, 80, 237, 170, 278, 369, 214, 397, 612]
        self._chom = [numpy.random.random()*360 for i in range(len(self._path) + 1)]
        self._chom = [120.29406804220929, 173.13122303544387, 149.596315520899, 25.736667918754762, 22.65521404775494,
         353.8370829315566, 82.5783995134154, 149.79627814253115, 64.88160853869039, 320.4755500020353,
         112.70463715865714, 300.3106726280362, 90.86384261380607, 110.2368779686459, 175.53029291142224,
         191.37004772252465, 106.02538390791376, 228.13357461006942, 17.495778811648798, 155.31843986379695,
         333.79671052596984, 78.26576545978241, 128.3261805514433, 235.4921115848389, 203.59492196063988,
         207.37135634772216, 219.07897600625267, 244.5166620704587]

        # ins2
        self._chom = [345.2301533985212, 76.37103978382083, 66.7256904054031, 73.54898028369263, 59.62380874409132, 21.34382172793599, 320.4112747637505, 15.360384306741018, 74.01633670563832, 89.96639632153273, 5.608575917534566, 36.892217658963055, 1.6672847062868712, 11.812716664227864, 106.87519269637542, 95.89442952989626, 139.9507953165151, 103.24013075535129, 209.11139702429324, 239.9571289882257, 121.2727576867191, 208.92388908872638, 350.46003197301553, 157.29870263654922, 196.23832035033945, 187.2991375999456, 318.69248957087393, 5.963526156555421]

            # [219.08355731048113, 46.70097459212391, 151.0380130366836, 352.26123391345504, 24.590101231399405, 45.52232322497244, 22.24702590896815, 90.59745029483653, 29.181283472031495, 66.61702722225326, 3.2264239531397054, 106.02945264616024, 81.8978257103467, 25.460013022973374, 233.7300056999149, 198.25536418762036, 72.0395241368901, 83.75372878490414, 293.056482797359, 251.83376764661597, 294.5168713197628, 93.54023126657557, 182.28028961111445, 189.97139199432306, 186.832845880104, 39.25613522322138, 119.30578438644889, 120.16708445653799]

        print(self._chom)
        # self._path = []
    def getPathLength(self):
        return len(self._path)

    def evl(self,individual):

        t_dis = 0
        dirLst = []

        self._chom = individual
        init_psi = numpy.deg2rad(self._chom[0])
        init_gama = numpy.deg2rad((self._chom[1] - 180)/2)

        # print('init_gama',init_gama)

        dirPos_init = DirPoint3D(self._pntLst[0].x,
                             self._pntLst[0].y,
                             self._pntLst[0].z,
                             init_psi,
                             init_gama)
        sta = STADir(dir_pos= dirPos_init, er = ER(0,0))


        yawLst = []
        yawLst.append(0)
        for i in range(2,len(self._chom)):
            yawLst.append(numpy.deg2rad(self._chom[i]))


        d_x = []
        d_y = []
        d_z = []


        t_dis = 0

        pos_dir_x = []
        pos_dir_y = []
        pos_dir_z = []


        end_dir_x = []
        end_dir_y = []
        end_dir_z = []


        for ind in range(1,len(self._path)):



            goal_x = self._pntLst[self._path[ind]].x
            goal_y = self._pntLst[self._path[ind]].y
            goal_z = self._pntLst[self._path[ind]].z

            drawB = False
            px, py, pz, dis_d, a_psi, a_gama, p3_x, p3_y, p3_z = gdb.Gdubins3D(sta.dir_pos.x,sta.dir_pos.y,sta.dir_pos.z,
                             goal_x,goal_y,goal_z,
                             sta.dir_pos.psi,sta.dir_pos.gama,yawLst[ind],drawB)


            # print(dis_d)

            dirPos = DirPoint3D(self._pntLst[self._path[ind]].x,
                                 self._pntLst[self._path[ind]].y,
                                 self._pntLst[self._path[ind]].z,
                                 a_psi,
                                 a_gama)

            # print('ind = ',ind)

            if ind != (len(self._path) -1):
                valid, er, dubins_dis = self.calDubinsER(sta, dirPos, self._typeLst[self._path[ind]], dis_d)
            else:
                valid, er, dubins_dis = self.calDubinsGoalER(sta,dirPos,dis_d)


            for j in range(len(px)):
                d_x.append(px[j])
                d_y.append(py[j])
                d_z.append(pz[j])

            # dirPos = DirPoint3D(self._pntLst[self._path[ind]].x,
            #                      self._pntLst[self._path[ind]].y,
            #                      self._pntLst[self._path[ind]].z,
            #                      a_psi,
            #                      a_gama)

            if valid:
                t_dis += dubins_dis
                sta  = STADir(dir_pos = dirPos, er = er)
                pass
            else:
                # t_dis = sys.float_info.max
                # print('failed')
                return sys.float_info.max,

        if valid:
            print('success',t_dis)
            return  t_dis,
        else:
            # print('failed')
            return sys.float_info.max,
        return t_dis,

    def calDubinsER(self,sta:STADir,dir_pos :DirPoint3D, e_type,dis):
        # try:
        # print(sta.dir_pos,dir_pos)
        # dubins_dis = self.calDubinsDis(sta.dir_pos, dir_pos)
        dubins_dis = dis
        # except Exception as e:
        # print(e)
        # exit()
        e_h = dubins_dis/ 1000 + sta.er.e_h
        e_v = dubins_dis / 1000 + sta.er.e_v
        valid = False
        if e_type == CorType.Vertical:
            if e_v <= self._a1 and e_h <= self._a2:
                valid = True
                e_v = 0
        if e_type == CorType.Horizontal:
            if e_v <= self._b1 and e_h <= self._b2:
                valid = True
                e_h = 0
        return valid, ER(e_v=e_v, e_h=e_h),dubins_dis


    def calDubinsGoalER(self,sta:STADir,dir_pos :DirPoint3D,dis):
        # dubins_dis = self.calDubinsDis(sta.dir_pos, dir_pos)
        dubins_dis = dis
        e_h = dubins_dis/ 1000 + sta.er.e_h
        e_v = dubins_dis / 1000 + sta.er.e_v
        valid = False
        print(e_h)
        print(e_v)
        if e_h < self._th and e_v < self._th:
            valid = True
        return valid, ER(e_v=e_v, e_h=e_h),dubins_dis

    def calDubinsDis(self, pos_a: DirPoint3D, pos_b: DirPoint3D):
        raise Exception('xx')
        return dis


    def drawDubinsPath(self):

        p_data = open('.//data//ins_' + str(case) + 'Best' + '角度' + 'pro2_922_s.dat', 'w')

        print(self._chom)
        print(self._path)
        # gp._path = [0, 503, 200, 80, 237, 170, 278, 369, 214, 397, 612]
        # # gp._chrom = [51.12083145920054, 178.6546480374125, 96.53216136306273, 65.68120520420554, 339.2745239620836, 29.692324175129926, 357.1342339082977, 13.96126135651108, 24.67063637610153, 43.081945140688966, 343.72373332373553, 55.53416525291733, 294.97751482614825, 160.55714352603155, 335.62029387958466, 135.1113892974001, 272.2317633544588, 115.50965977974849, 131.70937812167222, 255.65689160142625, 174.68614612015506, 1.1100793596630467]
        # gp._chrom = [35.64144355095377, 136.07851338455714, 351.42728912239573, 73.16130147768374, 33.52748385753523,
        #              337.8683291201086, 14.20709305212051, 53.08119797734826, 70.25048824593296, 93.78034812463086,
        #              359.7118978558144, 29.566128322279766, 324.4248087013906, 267.3070360965535, 44.7340138926815,
        #              142.5291949602862, 30.810401439704037, 216.8063045595583, 311.32800296731324, 110.98127345792193,
        #              266.86503744086997, 201.95337987458723]

        self._chom = self._chrom
        # exit()
        # exit()
        psi_lst = []
        gama_lst = []

        init_psi = numpy.deg2rad(self._chom[0])
        init_gama = numpy.deg2rad((self._chom[1] - 180)/2)

        psi_lst.append(init_psi)
        gama_lst.append(init_gama)

        dirPos_init = DirPoint3D(self._pntLst[0].x,
                             self._pntLst[0].y,
                             self._pntLst[0].z,
                             init_psi,
                             init_gama)
        sta = STADir(dir_pos= dirPos_init, er = ER(0,0))



        # yawLst = []
        # yawLst.append(0)
        # for i in range(2,len(self._chom)):
        #     yawLst.append(self._chom[i]*math.pi*2)

        yawLst = []
        yawLst.append(0)
        for i in range(2, len(self._chom)):
            yawLst.append(numpy.deg2rad(self._chom[i]))



        # print(yawLst)

        d_x = []
        d_y = []
        d_z = []


        t_dis = 0

        pos_dir_x = []
        pos_dir_y = []
        pos_dir_z = []


        end_dir_x = []
        end_dir_y = []
        end_dir_z = []

        dis_d_lst = []
        for ind in range(1,len(self._path)):

            goal_x = self._pntLst[self._path[ind]].x
            goal_y = self._pntLst[self._path[ind]].y
            goal_z = self._pntLst[self._path[ind]].z

            drawB = False
            if ind > 5:
                drawB = False
                # break

            px, py, pz, dis_d, a_psi, a_gama, p3_x, p3_y, p3_z = gdb.Gdubins3D(sta.dir_pos.x,sta.dir_pos.y,sta.dir_pos.z,
                             goal_x,goal_y,goal_z,
                             sta.dir_pos.psi,sta.dir_pos.gama,yawLst[ind],drawB)
            dis_d_lst.append(dis_d)

            psi_lst.append(a_psi)
            gama_lst.append(a_gama)

            # exit()
            pos_dir_x.append(sta.dir_pos.x + 1000*math.cos(sta.dir_pos.psi))
            pos_dir_y.append(sta.dir_pos.y + 1000*math.sin(sta.dir_pos.psi))
            pos_dir_z.append(sta.dir_pos.z + 1000*math.sin(sta.dir_pos.gama))


            end_dir_x.append(goal_x + 1000*math.cos(a_psi)*math.cos(a_gama))
            end_dir_y.append(goal_y + 1000*math.sin(a_psi)*math.cos(a_gama))
            end_dir_z.append(goal_z + 1000*math.sin(a_gama))


            print(1000*math.cos(a_psi)*math.cos(a_gama))
            print(1000*math.sin(a_psi)*math.cos(a_gama))
            print(1000*math.sin(a_gama))


            print('a_psi',a_psi)
            print('a_gama', a_gama)

            # d_x.extend(px)
            # d_y.extend(py)
            # d_z.extend(pz)

            # print(px)
            # print(py)
            # print(py)
            # print(pz)
            # print(px)
            # print(len(px))
            # print(px[1])

            for j in range(len(px)):
                d_x.append(px[j])
                d_y.append(py[j])
                d_z.append(pz[j])


            # if ind >= 8:
            #     # exit()
            #     break
            # print(d_x)
            # exit()
            # print(d_y)
            # print(d_z)

            # break
            # exit()

            # print(goal_dir)
            # exit()
            dirPos = DirPoint3D(self._pntLst[self._path[ind]].x,
                                 self._pntLst[self._path[ind]].y,
                                 self._pntLst[self._path[ind]].z,
                                 a_psi,
                                 a_gama)
            print('draw ', ind)
            sta = STADir(dirPos, ER(0,0))

        print(sum(dis_d_lst))
        print(sum(dis_d_lst))
        # exit()

        rd.writeConf(p_data,'psi',psi_lst)
        rd.writeConf(p_data,'gama',gama_lst)
        exit()

        self._scatterLst = []
        v_x = []
        v_y = []
        v_z = []

        h_x = []
        h_y = []
        h_z = []

        for i in range(len(self._pntLst)):
            if self._typeLst[i] == CorType.Vertical:
                v_x.append(float(self._pntLst[i].x))
                v_y.append(float(self._pntLst[i].y))
                v_z.append(float(self._pntLst[i].z))
            else:
                h_x.append(float(self._pntLst[i].x))
                h_y.append(float(self._pntLst[i].y))
                h_z.append(float(self._pntLst[i].z))

        # s_x = []
        # s_y = []
        # s_z = []
        #
        # for ind in self._tree:
        #     s_x.append(self._pntLst[ind].x)
        #     s_y.append(self._pntLst[ind].y)
        #     s_z.append(self._pntLst[ind].z)

        p_x = []
        p_y = []
        p_z = []

        for ind in self._path:
            p_x.append(self._pntLst[ind].x)
            p_y.append(self._pntLst[ind].y)
            p_z.append(self._pntLst[ind].z)



        f_data = open('.//data//ins_' + str(case) + 'Best' + 'rs' + 'pro2_922_s.dat', 'w')


        m_data = open('.//data//ins_' + str(case) + 'Best' + '局部' + 'pro2_922_s.dat', 'w')


        min_x = []
        min_y = []
        min_z = []
        print(self._path[-2])
        for i in range(0,len(d_x)):
            if gdb.distance([d_x[i],d_y[i],d_z[i]],[self._pntLst[self._path[-2]].x,
                                                      self._pntLst[self._path[-2]].y,
                                                      self._pntLst[self._path[-2]].z]) < 200:
                min_x.append(d_x[i])
                min_y.append(d_y[i])
                min_z.append(d_z[i])

        rd.writeConf(m_data,'x',min_x)
        rd.writeConf(m_data,'y',min_y)
        rd.writeConf(m_data,'z',min_z)


        w_x = []
        w_y = []
        w_z = []
        for i in range(0,len(d_x),10):
            w_x.append(d_x[i])
            w_y.append(d_y[i])
            w_z.append(d_z[i])

        rd.writeConf(f_data,'x',w_x)
        rd.writeConf(f_data,'y',w_y)
        rd.writeConf(f_data,'z',w_z)
        f_data.close()

        f_data = open('.//data//ins_' + str(case) + 'Best' + 'rs' + 'dis_pro2_922_s.dat', 'w')
        rd.writeConf(f_data,'dis',dis_d_lst)
        f_data.close()


        # self._scatterLst.append(go.Scatter3d(x = v_x ,y = v_y , z = v_z, mode = 'markers', marker=dict(size = 8), name = '垂直'))
        # self._scatterLst.append(go.Scatter3d(x = h_x ,y = h_y , z = h_z, mode = 'markers', marker=dict(size = 8),name = '水平'))

        self._scatterLst.append(go.Scatter3d(x = [float(self._pnt_A.x)] ,y = [float(self._pnt_A.y)] , z = [float(self._pnt_A.z)],
                                             mode = 'markers', marker = dict(size = 15),name = 'A'))
        self._scatterLst.append(go.Scatter3d(x = [float(self._pnt_B.x)] ,y = [float(self._pnt_B.y)] , z = [float(self._pnt_B.z)],
                                             mode = 'markers',marker = dict(size = 15),name = 'B'))

        self._scatterLst.append(go.Scatter3d(x = d_x, y = d_y, z = d_z, mode='lines', line= dict(width = 3) , name='dUBINS'))
        self._scatterLst.append(go.Scatter3d(x=p_x, y=p_y, z=p_z, mode='lines', line= dict(width = 3), name='path'))


        for i in range(len(pos_dir_x)):
            self._scatterLst.append(go.Scatter3d(x=[self._pntLst[self._path[i]].x, pos_dir_x[i] ],
                                                 y=[self._pntLst[self._path[i]].y, pos_dir_y[i] ],
                                                 z=[self._pntLst[self._path[i]].z, pos_dir_z[i] ], mode='lines', name = 'start',line=dict(width=5)))


        for i in range(len(pos_dir_x)):
            self._scatterLst.append(go.Scatter3d(x=[self._pntLst[self._path[i + 1]].x, end_dir_x[i] ],
                                                 y=[self._pntLst[self._path[i + 1]].y, end_dir_y[i] ],
                                                 z=[self._pntLst[self._path[i + 1]].z, end_dir_z[i] ], mode='lines', name = 'end', line=dict(width=5)))


        fig = go.Figure(data=self._scatterLst)
        plotly.offline.plot(fig, filename='test_ins_1_dubins.html')


case = 2

# case1 = True
if case == 1:
    read_cfg = rd.Read_Cfg('.//data//ins_1.dat')
    print('ins 1')
    '''
    ins = 1
    '''
    a1 = 25
    a2 = 15
    b1 = 20
    b2 = 25
    th = 30
else:
    read_cfg = rd.Read_Cfg('.//data//ins_2.dat')
    a1 = 20
    a2 = 10
    b1 = 15
    b2 = 20
    th = 20

# lst_x = []


lst_x = []
lst_y = []
lst_z = []
_typeLst = []

allPntLst = []
pntLst = []
typeLst = []

read_cfg.get('x', lst_x)
read_cfg.get('y', lst_y)
read_cfg.get('z', lst_z)
read_cfg.get('type', _typeLst)
read_cfg.get('pnt_A', pntLst)
pnt_A = Point3D(pntLst[0], pntLst[1], pntLst[2])
allPntLst.append(pnt_A)
typeLst.append(CorType.Start)
pntLst = []
for i in range(len(lst_x)):
    allPntLst.append(Point3D(lst_x[i], lst_y[i], lst_z[i]))
    if _typeLst[i] == 1:
        typeLst.append(CorType.Vertical)
    else:
        typeLst.append(CorType.Horizontal)
read_cfg.get('pnt_B', pntLst)
pnt_B = Point3D(pntLst[0], pntLst[1], pntLst[2])
allPntLst.append(pnt_B)
typeLst.append(CorType.Goal)

# print(len(allPntLst))
# print(allPntLst)
gp = Pro_Solver(allPntLst, typeLst, lst_x, lst_y, lst_z, pnt_A, pnt_B, a1, a2, b1, b2, th)
if case == 1:
    gp._path = [0, 503, 200, 80, 237, 170, 278, 369, 214, 397, 612]
    # gp._chrom = [51.12083145920054, 178.6546480374125, 96.53216136306273, 65.68120520420554, 339.2745239620836, 29.692324175129926, 357.1342339082977, 13.96126135651108, 24.67063637610153, 43.081945140688966, 343.72373332373553, 55.53416525291733, 294.97751482614825, 160.55714352603155, 335.62029387958466, 135.1113892974001, 272.2317633544588, 115.50965977974849, 131.70937812167222, 255.65689160142625, 174.68614612015506, 1.1100793596630467]
    gp._chrom = [35.64144355095377, 136.07851338455714, 351.42728912239573, 73.16130147768374, 33.52748385753523, 337.8683291201086, 14.20709305212051, 53.08119797734826, 70.25048824593296, 93.78034812463086, 359.7118978558144, 29.566128322279766, 324.4248087013906, 267.3070360965535, 44.7340138926815, 142.5291949602862, 30.810401439704037, 216.8063045595583, 311.32800296731324, 110.98127345792193, 266.86503744086997, 201.95337987458723]
    # [267.7831627201636, 306.4932301908233, 90.84891682074073, 3.052894486920783, 316.3384433551848, 13.649951015489009, 51.92333488618941, 58.408031446590996, 68.46058573580746, 61.7461542639787, 97.05332542112909, 350.559085017489, 253.4483312388257, 183.19454858804056, 136.0687803636989, 124.8951184425438, 74.07423262609369, 242.69508512887109, 217.03152151382304, 69.88271219466826, 165.67789362799306, 239.74471018032574]

if case == 2:
    gp._path = [0, 163, 114, 8, 309, 305, 123, 45, 160, 92, 93, 61, 292, 326]
    gp._chrom = [46.12565061062902, 294.4297234381514, 95.63287802732546, 6.282776399890999, 320.3385844310986, 113.93515850679738, 112.92299036287079, 68.10531840903259, 316.1864318676872, 62.65642637577015, 345.94896879083797, 78.47137424886117, 64.9354244486568, 71.62211742550045, 45.47999421785155, 223.58241468228908, 59.25940702203084, 25.22969869511165, 149.28819541490984, 292.87246755095515, 97.52076352432084, 201.37722656714743, 117.59087570840806, 279.888841490533, 191.76788859196347, 345.53090219320904, 85.61106753832667, 122.42040444013737]
        # [219.08355731048113, 46.70097459212391, 151.0380130366836, 352.26123391345504, 24.590101231399405, 45.52232322497244, 22.24702590896815, 90.59745029483653, 29.181283472031495, 66.61702722225326, 3.2264239531397054, 106.02945264616024, 81.8978257103467, 25.460013022973374, 233.7300056999149, 198.25536418762036, 72.0395241368901, 83.75372878490414, 293.056482797359, 251.83376764661597, 294.5168713197628, 93.54023126657557, 182.28028961111445, 189.97139199432306, 186.832845880104, 39.25613522322138, 119.30578438644889, 120.16708445653799]
        # [345.2301533985212, 76.37103978382083, 66.7256904054031, 73.54898028369263, 59.62380874409132, 21.34382172793599, 320.4112747637505, 15.360384306741018, 74.01633670563832, 89.96639632153273, 5.608575917534566, 36.892217658963055, 1.6672847062868712, 11.812716664227864, 106.87519269637542, 95.89442952989626, 139.9507953165151, 103.24013075535129, 209.11139702429324, 239.9571289882257, 121.2727576867191, 208.92388908872638, 350.46003197301553, 157.29870263654922, 196.23832035033945, 187.2991375999456, 318.69248957087393, 5.963526156555421]
        #  [219.08355731048113, 46.70097459212391, 151.0380130366836, 352.26123391345504, 24.590101231399405, 45.52232322497244, 22.24702590896815, 90.59745029483653, 29.181283472031495, 66.61702722225326, 3.2264239531397054, 106.02945264616024, 81.8978257103467, 25.460013022973374, 233.7300056999149, 198.25536418762036, 72.0395241368901, 83.75372878490414, 293.056482797359, 251.83376764661597, 294.5168713197628, 93.54023126657557, 182.28028961111445, 189.97139199432306, 186.832845880104, 39.25613522322138, 119.30578438644889, 120.16708445653799]

gp.drawDubinsPath()
exit()

# Problem dimension
NDIM = gp.getPathLength()

creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, typecode='d', fitness=creator.FitnessMin)

toolbox = base.Toolbox()
seed = 1
# random.seed(seed)

toolbox.register("attr_float", random.uniform, 0, 360)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, NDIM*2)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("select", tools.selRandom, k=3)
toolbox.register("evaluate", gp.evl)

print('xxx')

def main():
    # Differential evolution parameters

    CR = 0.25
    F = 1

    CR = 0.9
    F = 0.5


    CR = 0.8
    F = 0.7

    MU = 100
    NGEN = 100

    f_data = open('.//data//150_50_ins_1' + str(case) +'seed'+ 'F'+str(F)  + 'CR' + str(CR) + 'pro2_922_s.dat', 'w')


    e_data = open('.//data//150_50_fitness_ins_1' + str(case) +'seed'+ 'F'+str(F) + 'CR' + str(CR)+ 'pro2_922_s.dat', 'w')



    pop = toolbox.population(n=MU);
    hof = tools.HallOfFame(1)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)

    logbook = tools.Logbook()
    logbook.header = "gen", "evals", "std", "min", "avg", "max"


    min_fitness = sys.float_info.max
    min_ind = []
    # Evaluate the individuals
    fitnesses = toolbox.map(toolbox.evaluate, pop)
    for ind, fit in zip(pop, fitnesses):
        print(ind)
        lst = []
        for x in ind:
            lst.append(x)
        print('xxx = ',lst)

        ind.fitness.values = fit
        # if ind.fitness.values
    # exit()

    record = stats.compile(pop)
    logbook.record(gen=0, evals=len(pop), **record)
    print(logbook.stream)

    for g in range(1, NGEN):
        for k, agent in enumerate(pop):
            a, b, c = toolbox.select(pop)
            y = toolbox.clone(agent)
            index = random.randrange(NDIM)
            for i, value in enumerate(agent):
                if i == index or random.random() < CR:
                    y[i] = a[i] + F * (b[i] - c[i])
                    if y[i] > 360 or y[i] <0:
                        y[i] = np.random.random()*360
            y.fitness.values = toolbox.evaluate(y)
            if y.fitness > agent.fitness:
                pop[k] = y
        hof.update(pop)
        record = stats.compile(pop)
        logbook.record(gen=g, evals=len(pop), **record)
        print(logbook.stream)

        best_ind = tools.selBest(pop, 1)[0]
        print("Best individual is %s, %s" % (best_ind, best_ind.fitness.values))
        f_data.write(str(best_ind) + '\n')
        f_data.write(str(best_ind.fitness.values)+'\n')
        e_data.write(str(best_ind.fitness.values[0]) + '\n')
        e_data.flush()
        f_data.flush()

    print("Best individual is ", hof[0], hof[0].fitness.values[0])
    # f_data.write(hof[0])
    f_data.flush()
    # rd.writeConf(f_data,' - ',hof[0])
    f_data.close()


if __name__ == "__main__":
    main()
