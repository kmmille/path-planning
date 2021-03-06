#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Path Planner
# Author: Tianchen Ji
# Email: tj12@illinois.edu
# Create Date: 2019-11-26
# Modify Date: 2020-06-07
# ---------------------------------------------------------------------------

import sys
import copy
import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from math import sqrt
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import csv
from sensor import *
from navigator_new import *
import time

def onClick(event):
    global pause
    pause ^= True

stop = False

def animate(i):

    global cur_pos, dist_to_goal, time_stamp, goal_num
    global stop

    thres = 1
    # if goal_num == 0:
    #     thres = 1

    # if not pause:
        # get predicted future states of the agents
    agent_pos_pred, sense_ped = sense_peds(cur_pos, pedNum, agent_paths, time_stamp, lookahead_step_num, d_sense)
        # print(goal_num)

    if dist_to_goal <= thres and goal_num < len(goal_points):
        stop = True
        goal_num += 1
        print('time:',(lookahead_step_timeinterval)*time_stamp)
        if goal_num == 2:
            stop = True
    if not stop:
        end_point = goal_points[goal_num]
        # dist_to_goal =  sqrt((cur_pos[0] - end_point[0])**2 + (cur_pos[1] - end_point[1])**2)
        if goal_num < len(goal_points):
            # print(dist_to_goal)
            # convert from DM to float
            cur_pos = list(map(float, cur_pos))

            # plot robot position
            current_pos.set_data(cur_pos[0], cur_pos[1])
            # x_bound = [cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
            #            cur_pos[0] + 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
            #            cur_pos[0] - 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
            #            cur_pos[0] - 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
            #            cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2])]
            # y_bound = [cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
            #            cur_pos[1] + 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
            #            cur_pos[1] - 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
            #            cur_pos[1] - 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
            #            cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),]
            # car_outline.set_data(x_bound, y_bound)

            # plot agent positions
            # TODO:
            agent_pos_x = []
            agent_pos_y = []

            for j in range(pedNum):
                agent_pos_x.append(agent_paths[j][time_stamp][0])
                agent_pos_y.append(agent_paths[j][time_stamp][1])

            agent_pos.set_data(agent_pos_x, agent_pos_y)
            # print(len(sense_ped))
            sensed.set_data([ped[0] for ped in sense_ped], [ped[1] for ped in sense_ped])

            # solve for optimal control actions
            max_v = 1.5
            safety_r = 1.2
            # end_point = goal_points[goal_num]
            mpc_ = MPC(end_point=end_point,
                       num_of_agent=len(agent_pos_pred),
                       safety_r=safety_r,
                       max_v=max_v,
                       lookahead_step_num=lookahead_step_num,
                       lookahead_step_timeinterval=lookahead_step_timeinterval,
                       xlim = xlims[goal_num],
                       ylim = ylims[goal_num])
            t1 = time.time()
            sol       = mpc_.Solve(cur_pos, agent_pos_pred)
            t_syn = time.time() - t1
            v_opt     = sol['x'][3 * lookahead_step_num]
            delta_opt = sol['x'][4 * lookahead_step_num - 1]

            # with open('MPC_02_straight_1s.csv', 'a+') as csvfile:
            #     spamwriter = csv.writer(csvfile, delimiter=',')
            #     spamwriter.writerow([cur_pos[0], cur_pos[1], t_syn, time_stamp*lookahead_step_timeinterval, len(agent_pos_pred)])

            # simulate forward
            cur_pos[0] = cur_pos[0] + v_opt * lookahead_step_timeinterval * cos(cur_pos[2])
            cur_pos[1] = cur_pos[1] + v_opt * lookahead_step_timeinterval * sin(cur_pos[2])
            cur_pos[2] = cur_pos[2] + v_opt * lookahead_step_timeinterval * tan(delta_opt)

            dist_to_goal = sqrt((cur_pos[0] - end_point[0])**2 + (cur_pos[1] - end_point[1])**2)


            time_stamp += 1
        return current_pos, agent_pos, sensed

if __name__ == '__main__':
        filename = 'intersection_02_traj_ped.csv'
        pedNum = 120
        agent_paths = createPedPath(filename, pedNum)
        # print(agent_paths)
        time_stamp = 1
        pause = False

        cur_pos = [122,6,0]
        time_step = 0
        lookahead_step_num = int(1/(0.1))
        d_sense = 10
        lookahead_step_timeinterval = 0.1# 64e-3
        dist_to_goal = 100
        # end_point = [136, 5]
        goal_points = [[142.4, 3.5]]#, [138,20]]
        xlims = [None]#, [132, 139]]
        ylims = [[1, 7]]#, None]
        goal_num = 0
        max_v = 3

        fig, ax = plt.subplots(figsize=(7, 7))
        fig.canvas.mpl_connect('button_press_event', onClick)

        patches = [Rectangle((113, 9), 19, 21), Rectangle((113, -22), 19, 21), Rectangle((141, -22), 19, 21), Rectangle((141, 9), 19, 21)]
        pc = PatchCollection(patches, facecolor='red')
        ax.add_collection(pc)
        ax.plot([131,138], [5, 20], ls = 'None', color = 'g')


        current_pos, = plt.plot([], [] , ls='None', color='b', marker='o', label='car_pos')
        # car_outline, = plt.plot([], [] , color='b')
        agent_pos, = plt.plot([], [] , ls='None', color='k', marker='o', label='agent_pos')
        sensed, = plt.plot([], [] , ls='None', color='g', marker='o', label='agent_pos')

        ani = animation.FuncAnimation(fig, animate, frames=1000, interval=20)

        plt.axis([118, 145, -3, 30])
        plt.grid()
        plt.show()
