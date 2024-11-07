#-*- coding: utf-8 -*-
from __future__ import print_function

import argparse
import collections
import datetime
import glob
import logging
import math
import os
import numpy.random as random
import re
import sys
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc

import sys
sys.path.append("I:\\Tools\\carla\\WindowsNoEditor\\PythonAPI\\carla")
sys.path.append("I:\\Tools\\carla\\WindowsNoEditor\\PythonAPI\\carla\\agents")

from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.controller import VehiclePIDController
from agents.tools.misc import draw_waypoints, distance_vehicle, vector, is_within_distance, get_speed
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.controller import VehiclePIDController, PIDLongitudinalController

# 这里是连接Carla 服务器
client = carla.Client('localhost', 2000)
client.set_timeout(100.0)  # seconds

client.load_world('Town01')
world = client.get_world()

#dt = 0.05
#settings = world.get_settings()
#settings.fixed_delta_seconds = dt # Set a variable time-step
#world.apply_settings(settings)

m = world.get_map()
spawn_points = m.get_spawn_points()   #生成点

# global path planner
origin = carla.Location(spawn_points[150].location)  #定义起始点
destination = carla.Location(spawn_points[50].location)  #定义目标点

# spawn ego vehicle 生成主车，定义生成的点
blueprint_library = world.get_blueprint_library()   #蓝图库
ego_bp = blueprint_library.find('vehicle.tesla.model3')
ego = world.spawn_actor(ego_bp, spawn_points[150])

# 定义观察视角
spectator = world.get_spectator()    #设置观察视角
ego_transform = ego.get_transform()
#asshole spectator
'''transform1 = carla.Transform(ego_transform.transform(carla.Location(x=-6, z=2.5)),
                                ego_transform.rotation)'''
#overlooking spectator
transform1 = carla.Transform(ego_transform.location + carla.Location( z = 20),
     carla.Rotation(pitch = -90) )
spectator.set_transform(transform1)

distance = 1
grp = GlobalRoutePlanner(m, distance)   #全局路径规划算法

"""
This method returns list of (carla.Waypoint, RoadOption)
from origin to destination
"""
route = grp.trace_route(origin, destination)

wps = []    # waypoint的集合
for i in range(len(route)):
    wps.append(route[i][0])

"""
    Draw a list of waypoints at a certain height given in z.
        :param world: carla.world object
        :param waypoints: list or iterable container with the waypoints to draw
        :param z: height in meters
"""
draw_waypoints(world, wps)

#绘制点和曲线
for pi, pj in zip(route[:-1], route[1:]):      #[:-1]从第一个元素开始到倒数第二个，[1：]从第二个开始到最后，pi和pj刚好成连续的前后两个点
    #zip函数将两个数组打包为元组，((route[0],route[1]),(route[1],route[2]),...,)
    pi_location = pi[0].transform.location
    pj_location = pj[0].transform.location
    pi_location.z = 0.5
    pj_location.z = 0.5
    world.debug.draw_line(pi_location, pj_location, thickness=0.1,  color=carla.Color(r=255))  #life_time=T, 绘制线条
    pi_location.z = 0.6
    world.debug.draw_point(pi_location, color=carla.Color(g=255))#, life_time=T  绘制点

def condistance(location1,location2):
    dx = location1.x - location2.x
    dy = location1.y - location2.y
    dz = location1.z - location2.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)


#ego.apply_control(carla.VehicleControl(throttle=5.0, steer=0))
agent = BasicAgent(ego)
agent.set_destination(destination)

k = 0
epoch = 5000
acceleration = []
velocity = []
angular_velocity = []
while k < len(route):
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break

#PID参数设置
    initspd_pid = PIDLongitudinalController(ego, K_P=3, K_I=0.1, K_D=2.0)
    # 设置目标车的cut_in的横纵向控制PID
    args_lateral_dict = {'K_P': 1.0, 'K_D': 2.0, 'K_I': 0.0, "dt": 1.0/1.0}
    args_long_dict = {'K_P': 1.0, 'K_D': 2.0, 'K_I': 0.0, "dt": 1.0/1.0}
    PID = VehiclePIDController(ego, args_lateral_dict, args_long_dict)
    target_speed = 15

    control = PID.run_step(target_speed, wps[k])
    ego.apply_control(control)

    velocity.append(ego.get_velocity())
    acceleration.append(ego.get_acceleration())
    angular_velocity.append(ego.get_angular_velocity())

    target_location = wps[k].transform.location
    if condistance(ego.get_transform().location, target_location) <= 2.0:
        k += 1

    #epoch -= 1
    #if epoch == 0:
    #    break

# 画图
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号
v_x = []
v_y = []
v_z = []
a_x = []
a_y = []
a_z = []
ywd_x = []
ywd_y = []
ywd_z = []
for data in velocity:
    v_x.append(data.x)
    v_y.append(data.y)
    v_z.append(data.z)

for data in acceleration:
    a_x.append(data.x)
    a_y.append(data.y)
    a_z.append(data.z)

for data in acceleration:
    ywd_x.append(data.x)
    ywd_y.append(data.y)
    ywd_z.append(data.z)

velocity_xyz = [v_x, v_y, v_z]
plt.figure(1)
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(np.array(velocity_xyz[i]))
    plt.xlabel('时间节点')
    plt.ylabel('速度 m/s')
plt.show()


plt.figure(2)
acceleration_xyz = [a_x, a_y, a_z]
for i in range(3):
    plt.subplot(3,1, i+1)
    plt.plot(np.array(acceleration_xyz[i]))
    plt.xlabel('时间节点')
    plt.ylabel('加速度 m/s^2')
plt.show()

plt.figure(3)
angular_velocity_xyz = []
for i in range(3):
    plt.subplot(3, 1, i+1)
    plt.plot(np.array(acceleration_xyz[i]))
    plt.xlabel('时间节点')
    plt.ylabel('偏航角 rad/s')
plt.show()
