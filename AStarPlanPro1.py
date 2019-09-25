from abc import ABCMeta, abstractmethod
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

ER = namedtuple('ER', ['e_v', 'e_h'])

STA = namedtuple('STA',['pos','er'])

Point3D  = namedtuple('Point3D',['x','y','z'])

ED = namedtuple('ED',['sInd','tInd'])

ANode = namedtuple('ANode',['ind','er'])


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


Infinite = float('inf')

class Pro_Solver(object):

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

        def __str__(self):
            return str(self.data) + ' g = ' + str(self.gscore)  + 'f  = ' + str(self.fscore)

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = Pro_Solver.SearchNode(k)
            self.__setitem__(k, v)
            return v

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
        self.calDisMat()

        self._init_sta = STA(pos = self._pnt_A, er = ER(0,0))

        self._path = []
        self._pathLst = []

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from
        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def plan(self):
        start = ANode(0,er =ER(0,0))
        searchNodes = Pro_Solver.SearchNodeDict()
        startNode = searchNodes[start] = Pro_Solver.SearchNode(
            start, gscore= .0, fscore= self.heuristic_cost_estimate(start)
        )
        self.openSet = []
        heappush(self.openSet, startNode)

        searchTimes = 1
        while self.openSet:
            current = heappop(self.openSet)
            print(current)
            goal_valid, goal_er = self.calGoalER(STA(self._pntLst[current.data.ind],current.data.er))
            if goal_valid:
                path = self.reconstruct_path(current)
                for node in path:
                    self._path.append(node.ind)
                self._path.append(self._pntBind)
                print(self._path)
                break
                exit()
                raise  Exception('xx')
            current.out_openset = True
            current.closed = True
            for neighbor in [searchNodes[n] for n in self.neighbors(current.data)]:
                if neighbor.closed:
                    continue
                tentative_gscore = current.gscore + \
                    self.distance_between(current.data, neighbor.data)
                if tentative_gscore >= neighbor.gscore:
                    continue
                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                                  self.heuristic_cost_estimate(neighbor.data)
                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(self.openSet, neighbor)
            # self.displayOpenset()
            print('searchTimes = ', searchTimes)
            searchTimes += 1
            if searchTimes > 2000:
                exit()
            # print(openSet)
            # exit()
        print('xx')


    def displayOpenset(self):
        for i in self.openSet:
            print(i)


    def calGoalER(self,sta: STA):
        e_h = self.calDis(sta.pos, self._pnt_B) / 1000 + sta.er.e_h
        e_v = self.calDis(sta.pos, self._pnt_B) / 1000 + sta.er.e_v

        # if e_v == 20.2911741348684:
        #     raise Exception('xxx')

        if e_h < self._th and e_v < self._th:
            return True, ER(e_v = e_v, e_h = e_h)
        else:
            return False,ER(e_v = e_v, e_h = e_h)

    def calER(self, sta: STA, pos: Point3D, e_type):
        # e_h = self.calD(sta.pos, pos) / 1000 + sta.er.e_h
        # e_v = self.calV(sta.pos, pos) / 1000 + sta.er.e_v
        e_h = self.calDis(sta.pos, pos) / 1000 + sta.er.e_h
        e_v = self.calDis(sta.pos, pos) / 1000 + sta.er.e_v
        valid = False
        if e_type == CorType.Vertical:
            if e_v <= self._a1 and e_h <= self._a2:
                valid = True
                e_v = 0
        if e_type == CorType.Horizontal:
            if e_v <= self._b1 and e_h <= self._b2:
                valid = True
                e_h = 0
        return valid, ER(e_v=e_v, e_h=e_h)


    def heuristic_cost_estimate(self, node:ANode):
        dis = self.calDis(self._pntLst[node.ind],self._pnt_B)
        """computes the 'direct' distance between two (x,y) tuples"""
        return dis

    def distance_between(self, n1 :ANode, n2 :ANode):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        dis = self.calDis(self._pntLst[n1.ind],self._pntLst[n2.ind])
        return dis

    def neighbors(self, node:ANode):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        # aInd = ANode.ind

        resNeiLst = []

        for i in range(self._pntNum):
            valid, er = self.calER(STA(self._pntLst[node.ind],node.er),self._pntLst[i],self._typeLst[i])
            if valid:
                resNeiLst.append(ANode(ind = i,er = er))
        print(resNeiLst)
        return resNeiLst

    def __str__(self):
        return 'mcmp_astar row = '  + str(self._row) + ' col = ' + str(self._col)


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

    def calDis(self,pos_a :Point3D, pos_b:Point3D):
        dis = math.sqrt((pos_a.x- pos_b.x)**2 + (pos_a.y- pos_b.y)**2 + (pos_a.z- pos_b.z)**2)
        return dis

    def checkPath(self):
        _path = self._path
        sta = self._init_sta
        for i in range(1,len(_path)-1):
            ind = _path[i]
            valid, er = self.calER(sta, self._pntLst[ind], self._typeLst[ind])
            # print(ind,' ',self._tree.nodes[ind]['sta'].er)
            print(ind,' ',er)
            if valid == False:
                raise  Exception('error bug')
            sta = STA(pos = self._pntLst[ind],er = er)
        valid, er = self.calGoalER(sta)
        print('end er = ',er)
        # print(self._pntBind, ' ', self._tree.nodes[self._pntBind]['sta'].er)
        if valid == False:
            raise Exception('error bug')
    def calPathlength(self):
        print('Num = ',len(self._path))
        # print('Num = ',len(self._path))
        dis = 0
        for i in range(len(self._path)- 1):
            dis  += self.calDis(self._pntLst[self._path[i]],self._pntLst[self._path[i + 1]])
        dis += self.calDis(self._pntLst[self._path[-1]],self._pnt_B)
        print('dis = ',dis)
        return len(self._path),dis
if __name__ == '__main__':
    print(sys.argv)
    if len(sys.argv) == 3:
        if sys.argv[1] == '1':
            # exit()
            case = 1
        else:
            case = 2
        randSeed = int(sys.argv[2])
        np.random.seed(randSeed)
    else:
        randSeed = 100
        np.random.seed(randSeed)
        case = 1

    # case1 = True
    if  case == 1 :
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
    gp = Pro_Solver(allPntLst , typeLst, lst_x, lst_y, lst_z,pnt_A, pnt_B , a1 ,a2 ,b1 ,b2 ,th)

    gp.plan()
    gp.checkPath()
    gp.calPathlength()