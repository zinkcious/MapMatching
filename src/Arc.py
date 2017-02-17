#!/usr/bin/python
# -*- coding: UTF-8 -*-

class Arc:
    def __init__(self, argv):
        self.id = int(argv[0])
        self.from_node = int(argv[1])
        self.to_node = int(argv[2])
        self.len = float(argv[3])
        self.road_class = int(argv[4])
        self.geometry_list = [[float(ele) for ele in geo.strip().split(':')] for geo in argv[5].strip().split(';')]


