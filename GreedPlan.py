# import sympy as sp
import plotly.graph_objects as go
import numpy as np
from  enum import Enum
import plotly

import readcfg as rd
import networkx as nx
from collections import  namedtuple
import math
import sys

import matplotlib.pyplot as plt


ER = namedtuple('ER', ['e_v', 'e_h'])

STA = namedtuple('STA',['pos','er'])

Point3D  = namedtuple('Point3D',['x','y','z'])

ED = namedtuple('ED',['sInd','tInd'])


class Pnt:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
    def pnt2dict(self):
        dic = dict(x=x, y=y)
        return dic
    def display(self):
        print('x = ', self.x, 'y = ', self.y)


class Line:
    def __init__(self, pnt0=Pnt(), pnt1=Pnt()):
        self.x0 = pnt0.x
        self.y0 = pnt0.y
        self.x1 = pnt1.x
        self.y1 = pnt1.y

    def line2dict(self):
        dic = dict()
        dic['type'] = 'line'
        dic['x0'] = self.x0
        dic['y0'] = self.y0
        dic['x1'] = self.x1
        dic['y1'] = self.y1
        dic['line'] = dict(color='rgb(128, 0, 128)')
        return dic




class CorType(Enum):
    Vertical = 1
    Horizontal = 2
    Start = 3
    Goal = 4

class Greed_Plan(object):
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

        self._lst_x = lst_x
        self._lst_y = lst_y
        self._lst_z = lst_z

        self._min_x = min(self._lst_x)
        self._min_y = min(self._lst_y)
        self._min_z = min(self._lst_z)

        self._max_x = max(self._lst_x)
        self._max_y = max(self._lst_y)
        self._max_z = max(self._lst_z)

        self.calDisMat()
        self._init_sta = STA(pos = self._pnt_A, er = ER(0,0))
        self._tree = nx.DiGraph()
        self._openLst = []
        # nx.Graph
        self._tree.add_node(0,sta = self._init_sta)
        self.updateOpenList(0)
        np.random.seed(1)

        self._path = []

    def plan(self):
        # print(self._disMat)
        print('begin plan')
        # c_Pos = self._pnt_A
        # self.draw()
        searchTimes = 1
        while True:
            randPnt = self.getRandPos()
            # print(randPnt)
            disLst = []
            for ed,er in self._openLst:
                tPnt = self._pntLst[ed.tInd]
                dis = self.calDis(tPnt,randPnt)
                disLst.append(dis)

            if len(disLst)== 0:
                print('bug disLst')
                break
            minInd = disLst.index(min(disLst))
            min_ed, min_er = self._openLst[minInd]
            # print('searchTimes = ', searchTimes)
            min_sta = STA(pos = self._pntLst[min_ed.tInd], er = min_er)
            if min_ed.tInd in self._tree:
                print('min_ed = ', min_ed.tInd)
                raise Exception('xx')
            self._tree.add_node(min_ed.tInd, sta = min_sta)
            self._tree.add_edge(min_ed.tInd,min_ed.sInd)
            # print(self._tree.number_of_nodes())
            # raise Exception('xx')
            goal_valid, goal_er = self.calGoalER(min_sta)
            if goal_valid:
                self._tree.add_node(self._pntBind, sta = STA(self._pnt_B,goal_er))
                print('find path')
                self._path = [self._pntBind]
                ind = min_ed.tInd
                while True:
                    self._path.append(ind)
                    # print(self._path)
                    lst = list(self._tree.neighbors(ind))
                    if len(lst) == 0:
                        break
                    ind = lst[0]

                self._path = list(reversed(self._path))
                print(self._path)
                print(len(self._path))
                break
                # print(list(self._tree.neighbors(min_ed.tInd)))
                # raise Exception('xx')

            if min_ed.tInd == self._pntBind:
                raise Exception('xx')
            # print(self._openLst)
            self._openLst.remove(self._openLst[minInd])
            # print(self._openLst)
            self.updateOpenList(min_ed.tInd)

            searchTimes += 1
            if searchTimes > 1000:
                break
            if searchTimes != self._tree.number_of_nodes():
                print('searchTime = ',searchTimes)
                raise  Exception('ssssss')
        # nx.draw(self._tree)
        # plt.show()
        # print(len(self._))
        # for ind in self._tree:
        #     print(self._typeLst[ind])
        # self.drawTree()
        # self.draw()
        # raise Exception('xx')
        pass

    def earseOpenList(self,addInd):
        pass
        # while True:
        #     for

    def updateOpenList(self,addInd):
        disLst = list(self._disMat[addInd, :])

        '''
        earse
        '''
        earseLst = []
        for ind in range(len(self._openLst)):
            ed,er = self._openLst[ind]
            if ed.tInd == addInd:
                earseLst.append(self._openLst[ind])
        for unit in  earseLst:
            self._openLst.remove(unit)
        # for ed,er in self._openLst:
        #     ed.tInd
        # print(disLst)
        resLst = [(-1,sys.float_info.max,ER(-1,-1)) for i in range(5)]

        for i in range(self._pntNum):
            if i in self._tree:
                continue
            elif i == addInd:
                continue
            else:
                # print(resLst[0][1])
                # print(disLst[i])
                if resLst[0][1] > disLst[i]:
                    sta = self._tree.nodes[addInd]['sta']
                    if i == 521:
                        print('addInd', addInd)
                    valid,er = self.calER(sta,self._pntLst[i],self._typeLst[i])
                    if valid:
                        resLst[1] = (resLst[0][0],resLst[0][1],ER(resLst[0][2][0],resLst[0][2][1]))
                        resLst[0] = (i,disLst[i],er)
                        resLst = sorted(resLst, key = lambda  x: x[1])

        # print(resLst)
        for openInd,dis,er in resLst:
            if openInd == -1:
                continue
            self._openLst.append([ED(sInd= addInd, tInd= openInd),er])
        # print(self._openLst)
        # print(resLst)
        # seqLst = sorted(disLst)
        # print(seqLst)
        # print(disLst)
        # raise Exception('xx')


    def calGoalER(self,sta: STA):
        e_h = self.calH(sta.pos, self._pnt_B) / 1000 + sta.er.e_h
        e_v = self.calV(sta.pos, self._pnt_B) / 1000 + sta.er.e_v
        if e_h < 30 and e_v <30:
            return True, ER(e_v = e_v, e_h = e_h)
        else:
            return False,ER(e_v = e_v, e_h = e_h)


    def calER(self, sta:STA, pos: Point3D, e_type):
        e_h = self.calH(sta.pos, pos) / 1000 + sta.er.e_h
        e_v = self.calV(sta.pos, pos) / 1000 + sta.er.e_v
        valid = False
        if e_type == CorType.Vertical:
            if e_v <= self._a1 and e_h <= self._a2:
                valid = True
                e_v = 0
        if e_type == CorType.Horizontal:
            if e_v <= self._b1 and e_h <= self._b2:
                valid = True
                e_h = 0
        return valid,ER(e_v = e_v, e_h = e_h)

    #     STA.er.e_v
    # def find(self, self._init_sta , ):


    def getRandPos(self):
        rand_x = np.random.randint(self._min_x,self._max_x)
        rand_y = np.random.randint(self._min_y,self._max_y)
        rand_z = np.random.randint(self._min_z,self._max_z)

        return Point3D(rand_x, rand_y, rand_z)


        # print(self._pnt_A)
        # print(self._pnt_B)
        # print('xxx', self.calH(self._pnt_A,self._pnt_B))

    def calDisMat(self):
        self._disMat = np.zeros((len(self._pntLst), len(self._pntLst)))
        for i in range(len(self._pntLst)):
            for j in range (len(self._pntLst)):
                if i == j:
                    self._disMat[i][j] = 0
                else:
                    pa = self._pntLst[i]
                    pb = self._pntLst[j]
                    dis = math.sqrt((pa.x-pb.x)**2  + (pa.y-pb.y)**2 + (pa.z-pb.z)**2)
                    self._disMat[i][j] = dis
        return True

    '''
    计算水平距离
    '''
    def calH(self,pos_a :Point3D, pos_b:Point3D):
        # print(pnt2d_a)
        dis = math.sqrt((pos_a.x- pos_b.x)**2 + (pos_a.y- pos_b.y)**2)
        return dis
    '''
    计算垂直距离
    '''
    def calV(self,pos_a :Point3D, pos_b:Point3D):
        dis = abs(pos_b.z - pos_a.z)
        return dis
    '''
    计算垂直距离
    '''
    def calDis(self,pos_a :Point3D, pos_b:Point3D):
        dis = math.sqrt((pos_a.x- pos_b.x)**2 + (pos_a.y- pos_b.y)**2 + (pos_a.z- pos_b.z)**2)
        return dis


    def draw(self):

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

        s_x = []
        s_y = []
        s_z = []

        for ind in self._tree:
            s_x.append(self._pntLst[ind].x)
            s_y.append(self._pntLst[ind].y)
            s_z.append(self._pntLst[ind].z)


        # print(len(v_x))
        # raise Exception('XX')
        # print(h_x)
        # self._scatterLst.append(go.Scatter3d(x= [0,2], y=[0,3], z=[0,1]))

        # self._scatterLst.append(go.Scatter3d(x = v_x ,y = v_y , z = v_z, mode = 'markers', marker=dict(color = 'blue', size = 5), name = '垂直'))
        # self._scatterLst.append(go.Scatter3d(x = h_x ,y = h_y , z = h_z, mode = 'markers', marker=dict(color = 'orangered', size = 5),name = '水平'))
        self._scatterLst.append(go.Scatter3d(x = v_x ,y = v_y , z = v_z, mode = 'markers', marker=dict(size = 8), name = '垂直'))
        self._scatterLst.append(go.Scatter3d(x = h_x ,y = h_y , z = h_z, mode = 'markers', marker=dict(size = 8),name = '水平'))

        self._scatterLst.append(go.Scatter3d(x = [float(self._pnt_A.x)] ,y = [float(self._pnt_A.y)] , z = [float(self._pnt_A.z)],
                                             mode = 'markers', marker = dict(size = 15),name = 'A'))
        self._scatterLst.append(go.Scatter3d(x = [float(self._pnt_B.x)] ,y = [float(self._pnt_B.y)] , z = [float(self._pnt_B.z)],
                                             mode = 'markers',marker = dict(size = 15),name = 'B'))

        self._scatterLst.append(go.Scatter3d(x=s_x, y=s_y, z=s_z, mode='markers', marker=dict(size= 5), name='search'))
        fig = go.Figure(data=self._scatterLst)
        plotly.offline.plot(fig, filename='test_ins_2.html')



    def drawTree(self):
        lst = []
        for edge in self._tree.edges:
            # print(edge)
            s_x = self._pntLst[edge[0]].x
            s_y = self._pntLst[edge[0]].y
            t_x = self._pntLst[edge[1]].x
            t_y = self._pntLst[edge[1]].y

            lst.append((s_x,s_y,t_x,t_y))
            # raise Exception('xx')
            # Pos(self._pntLst)

        # for ind in self._tree:
        #     s_x.append(self._pntLst[ind].x)
        #     s_y.append(self._pntLst[ind].y)
        #     s_z.append(self._pntLst[ind].z)
        #
        _shapeLst = []
        mark_x = []
        mark_y = []
        for p in range(len(lst)):
            pnt0 = Pnt(lst[p][0], lst[p][1])
            pnt1 = Pnt(lst[p][2], lst[p][3])
            mark_x.append(pnt0.x)
            mark_x.append(pnt1.x)
            mark_y.append(pnt0.y)
            mark_y.append(pnt1.y)
            line = Line(pnt0, pnt1)
            lineDic = line.line2dict()
            #                print(randColor())
            lineDic['line']['color'] = 'darkred'
            # lineDic['line']['color'] = 'rgba(15,15,15,0.5)'
            lineDic['line']['width'] = 3
            _shapeLst.append(lineDic)
        #
        markTrace = go.Scatter(mode='markers',
                               x=mark_x,
                               y=mark_y,
                               marker=dict(size=3),
                               name='Spanning-Tree')
        _scatterLst = []
        _scatterLst.append(markTrace)
        layout = dict()
        layout['shapes'] = _shapeLst
        fig = go.Figure(data = _scatterLst, layout = layout)
        fig.show()
        # fig.show()
        # sp.Point2D.distance()

    def checkPath(self):
        sta = self._init_sta
        for i in range(1,len(self._path)-1):
            ind = self._path[i]
            valid, er = self.calER(sta, self._pntLst[ind], self._typeLst[ind])
            print(ind,' ',self._tree.nodes[ind]['sta'].er)
            print(ind,' ',er)
            if valid == False:
                raise  Exception('error bug')
            sta = STA(pos = self._pntLst[ind],er = er)
        valid, er = self.calGoalER(sta)
        print('end er = ',er)
        print(self._pntBind, ' ', self._tree.nodes[self._pntBind]['sta'].er)

        if valid == False:
            raise Exception('error bug')


if __name__ == '__main__':

    case1 = False
    if  case1 :
        read_cfg = rd.Read_Cfg('.//data//ins_1.dat')
        print('ins 1')
        '''
        ins = 1
        '''
        a1 = 20
        a2 = 10
        b1 = 15
        b2 = 20
        th = 20

    else:
        read_cfg = rd.Read_Cfg('.//data//ins_2.dat')
        a1 = 25
        a2 = 20
        b1 = 20
        b2 = 25
        th = 30
    # lst_x = []

    np.random.seed(1)

    lst_x = []
    lst_y = []
    lst_z = []
    _typeLst = []


    allPntLst = []
    pntLst = []
    typeLst = []

    read_cfg.get('x',lst_x)
    read_cfg.get('y',lst_y)
    read_cfg.get('z',lst_z)
    read_cfg.get('type',_typeLst)
    read_cfg.get('pnt_A', pntLst)
    pnt_A = Point3D(pntLst[0],pntLst[1],pntLst[2])
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
    pnt_B = Point3D(pntLst[0],pntLst[1],pntLst[2])
    allPntLst.append(pnt_B)
    typeLst.append(CorType.Goal)

    # print(len(allPntLst))
    # print(allPntLst)
    gp = Greed_Plan(allPntLst , typeLst, lst_x, lst_y, lst_z,pnt_A, pnt_B , a1 ,a2 ,b1 ,b2 ,th)
    gp.plan()
    gp.checkPath()

    # print(lst_x)

