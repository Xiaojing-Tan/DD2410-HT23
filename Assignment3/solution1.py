#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {Xiaojing Tan}
# {20010226-6798}
# {xta@kth.se}
# Reference: A* path planning (provided on Canvas)
# https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py

from dubins import *
import math
# import matplotlib.pyplot as plt

class Node:
    linear_res = 0.2
    angular_res = 1
    def __init__(self, car, x, y, theta, g_cost = 0.0, parent = None, step_control = [], phi = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.g_cost = g_cost
        self.h_cost = math.sqrt((x-car.xt)**2+(y-car.yt)**2)
        self.cost = self.g_cost + self.h_cost * 2
        self.parent = parent
        self.step_control = step_control
        self.phi = phi
    def renew_cost(self):
        self.cost = self.g_cost + self.h_cost * 2
    def get_index(self):
        return (int(self.x / self.linear_res), int(self.y / self.linear_res))

def check_target(car, x, y):
    if math.sqrt((x-car.xt)**2+(y-car.yt)**2) < 0.5:
        return True
    return False

def check_valid(car, x, y):
    if x < car.xlb:
        return False
    if x > car.xub:
        return False
    if y < car.ylb:
        return False
    if y > car.yub:
        return False
    for ob in car.obs:
        if math.sqrt((x - ob[0]) ** 2 + (y - ob[1]) ** 2) <= ob[2]+0.1:
            return False
    return True

def solution(car):
    # plot set up
    # boundx = [car.xlb, car.xub, car.xub, car.xlb, car.xlb]
    # boundy = [car.ylb, car.ylb, car.yub, car.yub, car.ylb]
    # plt.ion() # open iterative mode
    # plt.figure(1)
    # plt.plot(boundx,boundy)
    # plt.plot(car.x0, car.y0, 'kx')
    # plt.plot(car.xt, car.yt, 'kx')
    # for ob in car.obs:
    #     circle = plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k")
    #     plt.gca().add_patch(circle)
    
    controls, times = [], [0]
    v, timesteps, dt = 1, 80, 0.01
    start_node = Node(car, car.x0, car.y0, 0.0)
    open_set = {start_node.get_index(): start_node}
    closed_set = {}
    
    while open_set != {}:
        current_id = min(open_set, key = lambda index: open_set[index].h_cost)
        current_node = open_set.pop(current_id)
        if check_target(car, current_node.x, current_node.y):
            print("Reach target!")
            # plt.clf()
            while current_node.parent != None:
                controls = current_node.step_control + controls
                current_node = current_node.parent
            times = [x/100 for x in range(len(controls)+1)]
            break
        closed_set[current_id] = current_node
        # print("Check 1")
        for phi in [-math.pi/4, 0, math.pi/4]:
            valid = True
            xn, yn, thetan = current_node.x, current_node.y, current_node.theta
            for i in range(timesteps):
                xn, yn, thetan = step(car, xn, yn, thetan, phi)
                if not check_valid(car, xn, yn):
                    valid = False
                    break
            nbr_node = Node(car, xn, yn, thetan % (2 * math.pi))
            nbr_id = nbr_node.get_index()
            # print("Check 2")
            if nbr_id not in closed_set:
                if not valid:
                    closed_set[nbr_id] = nbr_node
                    continue
                nbr_node.g_cost = current_node.cost + v * timesteps * dt
                nbr_node.renew_cost()
                nbr_node.parent = current_node
                nbr_node.step_control = [phi] * timesteps
                # plt.plot([current_node.x, nbr_node.x],[current_node.y, nbr_node.y])
                # plt.axis("equal")
                # plt.show()
                # plt.pause(0.01)
                if nbr_id not in open_set:
                    # print("Check 3")
                    open_set[nbr_id] = nbr_node
                elif open_set[nbr_id].g_cost > nbr_node.g_cost:
                    # print("Check 4")
                    open_set[nbr_id] = nbr_node
    return controls, times