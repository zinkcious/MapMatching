#!/usr/bin/python
# -*- coding: UTF-8 -*-

from CarRecord import *
from Arc import *
from math import *
import Queue
import matplotlib.pyplot as plt
import sys
import time
import os
from datetime import datetime

def DistanceBetween(geo1, geo2):
    return pow(pow(float(geo1[0]) - float(geo2[0]), 2) + pow(float(geo1[1]) - float(geo2[1]), 2), 0.5)

# 知道三个顶点坐标ABC，看角ACB是否是锐角三角形
def CheckOxygon(A, C, B):
    a = DistanceBetween(C, B)
    b = DistanceBetween(A, C)
    c = DistanceBetween(A, B)
    if a**2 + b**2 < c**2:
        return False
    else:
        return True

def CalDirection(geo_list):
    if geo_list[-1][0] == geo_list[0][0]:
        if geo_list[-1][1] > geo_list[0][1]:
            return float(0)
        else:
            return float(180)
    else:
        slope = (geo_list[-1][1] - geo_list[0][1]) / (geo_list[-1][0] - geo_list[0][0])
        if geo_list[-1][0] > geo_list[0][0]: # 如果射线在第1,2象限
            return 90 - (atan(slope) / pi * 180) # 由于角度是跟y轴的夹角，所以需要90-
        else: # 如果射线在第3,4象限
            return 90 - (atan(slope) / pi * 180) + 180

# 传入geo1, geo2为直线y = kx + d上两点，求geo3在直线上的射影（即作垂直后与直线的交点）
# 如果形成钝角三角形，则垂点取geo1，geo2中靠近geo3的那个
def CalProjection(geo1, geo2, geo3):
    geo1 = [float(ele) for ele in geo1]
    geo2 = [float(ele) for ele in geo2]
    geo3 = [float(ele) for ele in geo3]
    a = DistanceBetween(geo2, geo3)
    b = DistanceBetween(geo1, geo3)
    c = DistanceBetween(geo1, geo2)
    if (a**2 + c**2) <= b**2: #钝角三角形，且geo3靠近geo2，包括点在线段延长线上的情况
        return geo2
    elif b**2 + c**2 <= a**2: #钝角三角形，且geo3靠近geo1，包括点在线段延长线上的情况
        return geo1
    elif a + b == c: # 说明点在线段上
        return geo3
    else:
        if geo1[0] == geo2[0]: # 如果直线竖直
            return [geo1[0], geo3[1]]
        elif geo1[1] == geo2[1]: # 如果直线水平
            return [geo3[0], geo1[1]]
        else:
            k = (geo1[1] - geo2[1]) / (geo1[0] - geo2[0]) # y = kx+d中的k
            d = geo1[1] - k * geo1[0] # y = kx+d中的d
            x4 = (k * geo3[1] - k * d + geo3[0]) / (1 + k**2)
            y4 = k * x4 + d
            return [x4, y4]

def CalProjectionOfArc(arc, geo):
    length = len(arc.geometry_list)
    # 先找一个虚拟点，其为点arc.geometry_list[-2]到arc.geometry_list[-1]连线延长线上延长相等距离的点
    virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
    # 找从哪个i开始，三角形ACB变成钝角，其中点A为car_record.geo,点C为arc.geometry_list[i]，点B为virtual_p
    i = 0
    while i < length - 1 and CheckOxygon(geo, arc.geometry_list[i], virtual_p):
        i += 1
    # 如果最小的i为0或者len() - 1，则取最靠近目标点的两个点计算垂点
    # 即使出现钝角三角形的情况，calProjection函数中会处理
    if i == 0:
        projection = CalProjection(arc.geometry_list[i], arc.geometry_list[i + 1], geo)
    else:  # 如果垂点在线段上
        projection = CalProjection(arc.geometry_list[i - 1], arc.geometry_list[i], geo)
    return projection

# 传入geo1, geo2为直线y = kx + d上两点，求geo3到直线的距离
# 如果形成钝角三角形，则垂点取geo1，geo2中靠近geo3的那个点与geo3的距离
def CalDistance(geo1, geo2, geo3):
    geo1 = [float(ele) for ele in geo1]
    geo2 = [float(ele) for ele in geo2]
    geo3 = [float(ele) for ele in geo3]
    a = DistanceBetween(geo2, geo3)
    b = DistanceBetween(geo1, geo3)
    c = DistanceBetween(geo1, geo2)
    if (a ** 2 + c ** 2) <= b ** 2:  # 钝角三角形，且geo3靠近geo2，包括点在线段延长线上的情况
        return DistanceBetween(geo2, geo3) * 5 # *5是geo点跑到外面去的panelty
    elif b ** 2 + c ** 2 <= a ** 2:  # 钝角三角形，且geo3靠近geo1，包括点在线段延长线上的情况
        return DistanceBetween(geo1, geo3) * 5
    elif a + b == c:  # 说明点在线段上
        return  0
    else:
        if geo1[0] == geo2[0]:  # 如果直线竖直
            return abs(geo3[0] - geo1[0])
        elif geo1[1] == geo2[1]:  # 如果直线水平
            return abs(geo3[1] - geo1[1])
        else:
            k = (geo1[1] - geo2[1]) / (geo1[0] - geo2[0])  # y = kx+d中的k
            d = geo1[1] - k * geo1[0]  # y = kx+d中的d
            dist = abs(k * geo3[0] + d - geo3[1]) / sqrt(1 + k**2) # 点到直线距离公式
            return dist

# 如果汽车定位数据跟道路的相差很远或者方向不对，则没有必要取该条arc
def CheckNecessary(arc, car_record):
    # 如果edgh方向和汽车的方向不同，则认为车不在该道路上
    if car_record.speed > 3 * 3.6:
        direction_road = CalDirection(arc.geometry_list)
        diff_angle = abs(car_record.direction - direction_road)
        if diff_angle > 180:
            diff_angle = 360 - diff_angle
        if diff_angle > 90:  # 如果道路方向和汽车方向不同
            return False

    # 我计算了该地图中最长的路是3494m，取大一点10000，防止出现漏判的情况，因为下面这个if是只取道路坐标的第一个点来跟目标点计算距离
    if DistanceActual(arc.geometry_list[0], car_record.geo) > 10000:
        return False

    # 下面if是计算这条道路上所有gps点跟目标点的最短距离小于100m才返回True。我们认为汽车gps精度为60m，放宽到100m
    min_dist = float("inf")
    for geo1 in arc.geometry_list:
        tmp = DistanceActual(geo1, car_record.geo)
        if tmp < min_dist:
            min_dist = tmp
    if min_dist < 300:
        return True
    else:
        return False

def CalCost(arc, car_record):
    cost = float("inf")
    if CheckNecessary(arc, car_record):
        length = len(arc.geometry_list)
        # 先找一个虚拟点，其为点arc.geometry_list[-2]到arc.geometry_list[-1]连线延长线上延长相等距离的点
        virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
        # 找从哪个i开始，三角形ACB变成钝角，其中点A为car_record.geo,点C为arc.geometry_list[i]，点B为virtual_p
        i = 0
        while i < length - 1 and CheckOxygon(car_record.geo, arc.geometry_list[i], virtual_p):
            i += 1
        # 如果最小的i为0或者len() - 1，则取最靠近目标点的两个点计算垂点
        # 即使出现钝角三角形的情况，calProjection函数中会处理
        if i == 0:
            dist = CalDistance(arc.geometry_list[i], arc.geometry_list[i + 1], car_record.geo)
            direction_road = CalDirection(arc.geometry_list[0:2])
        else: # 如果垂点在线段上
            dist = CalDistance(arc.geometry_list[i - 1], arc.geometry_list[i], car_record.geo)
            direction_road = CalDirection(arc.geometry_list[i - 1: i + 1])
        diff_angle = abs(car_record.direction - direction_road)
        if diff_angle > 180:
            diff_angle = 360 - diff_angle
        if car_record.speed < 1 * 3.6: #如果车速小于1，方向所占比重为0
            angle_ratio = 0
        elif car_record.speed < 10 * 3.6: #如果车速在1m/s到10m/s之间，随着车速增加，方向所占比重逐渐增加到0.54
            angle_ratio = (car_record.speed / 3.6 - 1) * 0.06
        else: #如果速度大于10m/s，方向所占比重保持0.54不变
            angle_ratio = 0.54
        cost = (dist / 0.00046 * 39.3) * (1 - angle_ratio) + (1.5 * diff_angle) * angle_ratio
    return cost

# 给定car_geo和car_direction，根据最短距离和方向，计算在所有道路的Cost，返回Cost最小的道路id
def CalMinCostArcID(car_record):
    min_cost = float("inf")
    id = 0
    for arc in arc_objects:
        cost = CalCost(arc, car_record)
        if cost < min_cost:
            min_cost = cost
            id = arc.id
    return id

#t1 - t2 yyyyMMddHHmmss
def SecondsBetween(t1, t2):
    if int(t1) > int(t2):
        big = t1
        small = t2
    else:
        big = t2
        small = t1
    big = time.strptime(big, "%Y%m%d%H%M%S")
    small = time.strptime(small, "%Y%m%d%H%M%S")
    big = datetime(big[0], big[1], big[2], big[3], big[4], big[5])
    small = datetime(small[0], small[1], small[2], small[3], small[4], small[5])
    return (big - small).seconds

# 从提供的道路数据的第1条可以知道经纬度的0.00046对应实际距离39.3m
def DistanceActual(geo1, geo2):
    dist = DistanceBetween(geo1, geo2)
    return dist * 9.745339101028061e4

# 把单行线用红蓝画出
def PlotMapRoad(arc_objects):
    record = {}
    for arc in arc_objects:
        ploted = False
        if record.has_key(arc.to_node):
            for ele in record[arc.to_node]:
                if ele == arc.from_node:
                    ploted = True
        nodeX = [geo[0] for geo in arc.geometry_list]
        nodeY = [geo[1] for geo in arc.geometry_list]
        if ploted == True:
            plt.plot(nodeX, nodeY, color='r', marker='.', alpha=0.3)
            plt.text(nodeX[0], nodeY[0], arc.id, alpha=0.3)
        else:
            plt.plot(nodeX, nodeY, color='#7f7f7f', marker='.', alpha=0.3)
            plt.text(nodeX[0], nodeY[0], arc.id, alpha=0.3)

        if record.has_key(arc.from_node):
            record[arc.from_node].append(arc.to_node)
        else:
            record[arc.from_node] = [arc.to_node]

# 输入arc_cover是arc_id和cover的列表，该函数输出这个arc_cover包含的道路总长
def CalLenCover(arc_cover):
    travel_distance = 0
    for k in range(len(arc_cover)):
        travel_distance += arc_objects[arc_cover[k][0] - 1].len * arc_cover[k][1]
    return round(travel_distance,1)

# 输入arc_list是arc_id的列表，该函数输出这个arc_list包含的道路总长
def CalLen(arc_list):
    travel_distance = 0
    for k in range(len(arc_list)):
        travel_distance += arc_objects[arc_list[k] - 1].len
    return round(travel_distance,1)

def findMinDist(dist, collected):
    min_dist = float("inf")
    min_key = -1
    for key in dist:
        if dist[key] < min_dist and (not collected.has_key(key)):
            min_dist = dist[key]
            min_key = key
    return min_key

# 用单源最短路径算法，求两条边的最短距离的arc_list
def dijkstra(graph, start_arc_id, end_arc_id):
    if start_arc_id == end_arc_id:
        return [start_arc_id]
    dist = {}
    path = {}
    collected = {}
    for key in graph:
        dist[key] = float("inf")
        for arc in graph[key]:
            if not dist.has_key(arc.to_node):
                dist[arc.to_node] = float("inf")
    from_node = arc_objects[start_arc_id - 1].to_node
    to_node = arc_objects[end_arc_id - 1].from_node
    if graph.has_key(from_node):
        for arc in graph[from_node]:
            dist[arc.to_node] = arc.len
            path[arc.to_node] = from_node
    dist[from_node] = 0
    collected[from_node] = True
    while True:
        node = findMinDist(dist, collected)
        if node == -1:
            break
        collected[node] = True
        if graph.has_key(node):
            for arc in graph[node]:
                if not collected.has_key(arc.to_node):
                    if dist[arc.to_node] > dist[node] + arc.len:
                        dist[arc.to_node] = dist[node] + arc.len
                        path[arc.to_node] = node
    arc_list = [end_arc_id]
    while path.has_key(to_node):
        for arc in graph[path[to_node]]:
            if arc.to_node == to_node:
                arc_list.append(arc.id)
        to_node = path[to_node]
    arc_list.append(start_arc_id)
    arc_list = arc_list[::-1]
    return arc_list

def BFSFindPathArc(arc, car_record, dist):
    min_cost = 60
    arc_list = []
    min_arc_id = 0
    visited = {}
    graph_around = {}
    q = Queue.Queue()
    q.put(arc)
    visited[arc.id] = True
    while not q.empty():
        arc1 = q.get()
        # 生成arc附近连通的道路的图，后面用来找最短路径
        if graph_around.has_key(arc1.from_node):
            graph_around[arc1.from_node].append(arc1)
        else:
            graph_around[arc1.from_node] = [arc1]
        cost = CalCost(arc1, car_record)
        if cost < min_cost:
            min_cost = cost
            min_arc_id = arc1.id
        if map_graph.has_key(arc1.to_node):
            for arc2 in map_graph[arc1.to_node]:
                if not visited.has_key(arc2.id):
                    if DistanceBetween(arc.geometry_list[-1], arc2.geometry_list[0]) < dist:
                        q.put(arc2)
                        visited[arc2.id] = True
    if min_arc_id != 0:
        arc_list = dijkstra(graph_around, arc.id, min_arc_id)
    return arc_list

# 计算点geo在arc上面的cover, flag="forward"时计算geo点垂点向前占arc的比例，flag="backward"时相反
def CalCover(arc, geo, flag):
    length = len(arc.geometry_list)
    virtual_p = [(2 * arc.geometry_list[-1][i] - arc.geometry_list[-2][i]) for i in range(2)]
    # 找从哪个i开始，三角形ACB变成钝角，其中点A为car_record.geo,点C为arc.geometry_list[i]，点B为virtual_p
    i = 0
    while i < length - 1 and CheckOxygon(geo, arc.geometry_list[i], virtual_p):
        i += 1
    # 如果最小的i为0或者len() - 1，则取最靠近目标点的两个点计算垂点
    # 即使出现钝角三角形的情况，calProjection函数中会处理
    if i == 0:
        projection = CalProjection(arc.geometry_list[i], arc.geometry_list[i + 1], geo)
    else:  # 如果垂点在线段上
        projection = CalProjection(arc.geometry_list[i - 1], arc.geometry_list[i], geo)
    cover = 0
    if flag == "forward":
        cover = round(DistanceBetween(projection, arc.geometry_list[-1]) \
                 / DistanceBetween(arc.geometry_list[0], arc.geometry_list[-1]), 2)
    elif flag == "backward":
        cover = round(DistanceBetween(projection, arc.geometry_list[0]) \
                      / DistanceBetween(arc.geometry_list[0], arc.geometry_list[-1]), 2)
    return cover

def CarRecordsDivide(car_i):
    index_start = sum(car_id_count[:car_i])
    index_end = sum(car_id_count[:car_i+1])
    if index_end - index_start == 1:
        car_i_start_end_index_list = [[index_start, index_end]]
    car_i_start_end_index_list = []
    index_j_start = index_start
    for i in range(index_start + 1, index_end):
        if SecondsBetween(car_record_objects[i - 1].time, car_record_objects[i].time) > 15*60:
            car_i_start_end_index_list.append([index_j_start, i])
            index_j_start = i
    car_i_start_end_index_list.append([index_j_start, index_end])
    return  car_i_start_end_index_list

def main():
    for car_i in range(0, car_num):
        print "Calculating car track of " + str(car_i + 1) + ", total " + str(car_num)
        output_file = open(result_file_name, 'a')
        # debug 可视化
        if PLOT == True:
            fig = plt.figure(car_i + 1)
        # 如果，前后两个定位点的时间超过15分钟，则认为前后可以分为两段轨迹，即不计算这两个定位点之间经过的道路
        car_i_start_end_index_list = CarRecordsDivide(car_i)
        for car_j in range(len(car_i_start_end_index_list)):
            car_ij_start_index = car_i_start_end_index_list[car_j][0] # 第i量车的第j段数据的第一个定位数据
            car_ij_end_index = car_i_start_end_index_list[car_j][1] # 第i量车的j段数据的第最后一个定位数据，不包括该条记录

            # arc_path存储着car_ij_start_index到car_ij_end_index之间经过的道路列表
            arc_path = []
            # 因为刚起步的时候汽车的方向不可信，所以取10米内的几个点的平均方向和坐标
            j = car_ij_start_index
            while j < car_ij_end_index:
                if DistanceActual(car_record_objects[j].geo, car_record_objects[car_ij_start_index].geo) > 10:
                    break
                j += 1
            car_record_list = car_record_objects[car_ij_start_index : j]
            car_geo_list = [car_record.geo for car_record in car_record_list]
            car_x = sum([geo[0] for geo in car_geo_list]) / len(car_geo_list)
            car_y = sum([geo[1] for geo in car_geo_list]) / len(car_geo_list)
            if len(car_geo_list) == 1:
                car_direction = car_record_list[0].direction
            else:
                car_direction = CalDirection(car_geo_list)
            car_record = CarRecord([0,0, car_x, car_y, 3*3.6+1, car_direction])
            arc_path.append(CalMinCostArcID(car_record))
            #算剩下的，基于前面算的结果，从跟前面道路相连通的道路里面找属于哪条道路
            while j < car_ij_end_index:
                car_record = car_record_objects[j]
                arc_list = BFSFindPathArc(arc_objects[arc_path[-1] - 1], car_record, DistanceBetween(car_record.geo, arc_objects[arc_path[-1] - 1].geometry_list[-1]))
                if len(arc_list) != 0:
                    arc_list.pop(0)
                    if len(arc_list) != 0:
                        arc_path = arc_path + arc_list
                else:
                    if len(arc_path) > 1:
                        arc_list = BFSFindPathArc(arc_objects[arc_path[-2] - 1], car_record, DistanceBetween(car_record.geo, arc_objects[arc_path[-2] - 1].geometry_list[-1]))
                        if len(arc_list) != 0:
                            arc_list.pop(0)
                            if len(arc_list) != 0:
                                arc_path.pop(-1)
                                arc_path = arc_path + arc_list
                        else:
                            arc_id = CalMinCostArcID(car_record)
                            if arc_id != 0 and arc_id != arc_path[-1]:
                                arc_path.append(arc_id)
                    else:
                        arc_id = CalMinCostArcID(car_record)
                        if arc_id != 0 and arc_id != arc_path[-1]:
                            arc_path.append(arc_id)
                j += 1

            # debug 可视化
            if PLOT == True:
                for arc_id in arc_path: # 画汽车经过的道路
                    nodex = [ele[0] for ele in arc_objects[arc_id - 1].geometry_list]
                    nodey = [ele[1] for ele in arc_objects[arc_id - 1].geometry_list]
                    plt.plot(nodex, nodey, color='#7f7f7f', marker='.', alpha=0.5)
                    plt.text((nodex[0] + nodex[1])/2, (nodey[0] + nodey[1])/2, arc_id, color='r', alpha=0.3)
                for j in range(car_ij_start_index, car_ij_end_index): # 画汽车gps坐标点
                    plt.scatter(car_record_objects[j].geo[0], car_record_objects[j].geo[1], marker='.', color='b', s=40, alpha=0.3)

            ## 下面这段代码计算car_ij_start_index到car_ij_end_index当中的每小段路经过了arc_path中的哪些arc
            if len(arc_path) == 1:
                projection_before = CalProjectionOfArc(arc_objects[arc_path[0] - 1], car_record_objects[car_ij_start_index].geo)
                for j in range(car_ij_start_index+1, car_ij_end_index):
                    # 去除重复GPS点
                    if car_record_objects[j].time == car_record_objects[j - 1].time:
                        continue
                    projection_now = CalProjectionOfArc(arc_objects[arc_path[0] - 1], car_record_objects[j].geo)
                    arc_cover = [[arc_path[0], round(DistanceActual(projection_before, projection_now) / arc_objects[arc_path[0] - 1].len, 4)]]
                    projection_before = projection_now # 更新projection_before的值
                    # 把arc_cover转换成要求格式，以便于输出
                    geo_output = []
                    for ele in arc_cover:
                        geo_output.append(':'.join([str(e) for e in ele]))
                    geo_output = '|'.join(geo_output)
                    # 输出
                    output_line = [str(car_i + 1), car_record_objects[j - 1].time, car_record_objects[j].time, str(CalLenCover(arc_cover)), \
                                   str(SecondsBetween(car_record_objects[j - 1].time,car_record_objects[j].time)), geo_output]
                    output_file.write(",".join(output_line) + "\n")
            else:
                # arc_path_i_before代表前一个arc在arc_path中的索引
                arc_path_i_before = 0
                for j in range(car_ij_start_index+1, car_ij_end_index):
                    # 去除重复GPS点
                    if car_record_objects[j].time == car_record_objects[j - 1].time:
                        continue
                    # dist 下一个定位点与上一个定位点之间的距离
                    dist = DistanceBetween(car_record_objects[j].geo, arc_objects[arc_path[arc_path_i_before] - 1].geometry_list[-1])
                    # arc_path_i_reachable表示与arc_path_i_before的距离小于dist的arc_path的索引
                    arc_path_i_reachable = arc_path_i_before + 1
                    while arc_path_i_reachable < len(arc_path):
                        if DistanceBetween(arc_objects[arc_path[arc_path_i_reachable] - 1].geometry_list[0], arc_objects[arc_path[arc_path_i_before] - 1].geometry_list[-1]) < dist:
                            arc_path_i_reachable += 1
                        else:
                            break
                    # 这样arc_path中arc_path_i_before到arc_path_i_reachable之间的arc就是j可能属于的arc
                    # 下面这段代码找arc_path_i_before到arc_path_i_reachable之间，j到底属于哪一条arc
                    min_cost = float('inf')
                    min_arc_path_i = arc_path_i_before
                    for arc_path_i in range(arc_path_i_before, arc_path_i_reachable):
                        cost = CalCost(arc_objects[arc_path[arc_path_i] - 1], car_record_objects[j])
                        if cost < min_cost:
                            min_cost = cost
                            min_arc_path_i = arc_path_i
                    # 找到min_arc_path_i是j属于的arc_path中的arc的索引
                    arc_path_output = arc_path[arc_path_i_before:min_arc_path_i+1] # 从j-1到j经过的arc列表
                    arc_path_output_cover = [[arc_id, 1.0] for arc_id in arc_path_output] # 从j-1到j经过的arc列表，带cover
                    # 计算首尾的cover
                    if len(arc_path_output_cover) == 1:
                        projection1 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j].geo)
                        projection2 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j - 1].geo)
                        arc_path_output_cover[0][1] = round(DistanceActual(projection1, projection2) / arc_objects[arc_path_output_cover[0][0] - 1].len, 4)
                    else:
                        projection1 = CalProjectionOfArc(arc_objects[arc_path_output_cover[0][0] - 1], car_record_objects[j - 1].geo)
                        projection2 = CalProjectionOfArc(arc_objects[arc_path_output_cover[-1][0] - 1], car_record_objects[j].geo)
                        arc_path_output_cover[0][1] = round(DistanceActual(projection1, arc_objects[arc_path_output_cover[0][0] - 1].geometry_list[-1]) \
                                                            / arc_objects[arc_path_output_cover[0][0] - 1].len, 4)
                        arc_path_output_cover[-1][1] = round(DistanceActual(projection2, arc_objects[arc_path_output_cover[-1][0] - 1].geometry_list[0]) \
                                                            / arc_objects[arc_path_output_cover[-1][0] - 1].len, 4)
                    # geo_output为处理arc_path_output_cover的数据，以便于输出
                    geo_output = []
                    for ele in arc_path_output_cover:
                        geo_output.append(':'.join([str(e) for e in ele]))
                    geo_output = ';'.join(geo_output)
                    # 输出
                    output_line = [str(car_i + 1), car_record_objects[j - 1].time, car_record_objects[j].time,str(CalLenCover(arc_path_output_cover)), \
                                   str(SecondsBetween(car_record_objects[j - 1].time, car_record_objects[j].time)), geo_output]
                    output_file.write(",".join(output_line) + "\n")
                    # 更新arc_path_i_before的值
                    arc_path_i_before = min_arc_path_i
        output_file.close()
        # debug 可视化图保存
        if PLOT == True:
            slash_index = result_file_name.rfind('/')
            if slash_index != -1:
                pic_dir = result_file_name[0:slash_index] + '/pic/'
            else:
                pic_dir = 'pic/'
            if not os.path.exists(pic_dir):
                os.mkdir(pic_dir)
            ax = plt.gca()
            ax.set_aspect(1)
            fig.savefig(pic_dir + str(car_i + 1))
            fig.clf()
            plt.close()

    return


if len(sys.argv) != 4:
    print '输入参数数目不对'
    exit()

map_file = open(sys.argv[1])
map_lines = map_file.readlines()
map_lines.pop(0)  # 删除第一行，为说明，非数据
# arc_objects存储着所有边的Arc对象实例，改成Arc不用Edgh命名是因为，Arc是有向边，Edgh是无向边
# 注意此处arc_objects列表中元素Arc的id比对应index大1，因为Arc的id从1开始，list的index从0开始
arc_objects = [Arc(argv.strip().split(',')) for argv in map_lines]

car_file = open(sys.argv[2])
car_lines_str = car_file.readlines()
# car_record_objects存储着汽车的每一条gps定位数据
car_record_objects = [CarRecord(line_str.strip().split(',')) for line_str in car_lines_str]
# car_id_count存储着从1到100每一辆车的gps定位数量
car_id = [car_record.car_id for car_record in car_record_objects]
car_num = max(car_id)
car_id_count = [car_id.count(i + 1) for i in range(car_num)]  # 100辆车

# 生成图的邻接表数据结构
map_graph = {}
for arc in arc_objects:
    if map_graph.has_key(arc.from_node):
        map_graph[arc_objects[arc.id - 1].from_node].append(arc)
    else:
        map_graph[arc_objects[arc.id - 1].from_node] = [arc]

# 输出文件名字
result_file_name = sys.argv[3]
if os.path.exists(result_file_name):
    os.remove(result_file_name)

# 是否画图
PLOT = False

if __name__ == '__main__':
    start = time.clock()
    main()
    end = time.clock()
    print "Spent " + str(int(end - start)) + " seconds"