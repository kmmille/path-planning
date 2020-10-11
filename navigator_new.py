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

def onClick(event):
    global pause
    pause ^= True

class FirstStateIndex:
    def __init__(self, n):
        self.px = 0
        self.py = self.px + n
        self.theta = self.py + n
        self.v = self.theta + n
        self.delta = self.v + n - 1

class MPC:
    def __init__(self, end_point, num_of_agent, safety_r, max_v=0.3, lookahead_step_num=5, lookahead_step_timeinterval=0.1):

        # The num of MPC actions, here include vx and vy
        NUM_OF_ACTS = 2

        # The num of MPC states, here include px and py
        NUM_OF_STATES = 3

        NUM_OF_G_STATES = num_of_agent

        self.end_point = end_point
        self.num_of_agent = num_of_agent
        self.safety_r = safety_r
        self.max_v = max_v
        self.lookahead_step_num = lookahead_step_num
        self.lookahead_step_timeinterval = lookahead_step_timeinterval
        self.first_state_index_ = FirstStateIndex(self.lookahead_step_num)
        self.num_of_x_ = NUM_OF_STATES * self.lookahead_step_num + NUM_OF_ACTS * (self.lookahead_step_num - 1)
        self.num_of_g_ = NUM_OF_STATES * self.lookahead_step_num + NUM_OF_G_STATES * self.lookahead_step_num

    def Solve(self, state, agent_state_pred):

        # define optimization variables
        x = SX.sym('x', self.num_of_x_)

        # define cost functions
        w_cte = 10.0
        w_dv = 1.0
        cost = 0.0

        # initial variables
        x_ = [0] * self.num_of_x_
        x_[self.first_state_index_.px:self.first_state_index_.py]    = [state[0]] * self.lookahead_step_num
        x_[self.first_state_index_.py:self.first_state_index_.theta] = [state[1]] * self.lookahead_step_num
        x_[self.first_state_index_.theta:self.first_state_index_.v]  = [state[2]] * self.lookahead_step_num
        x_[self.first_state_index_.v:self.first_state_index_.delta]  = [self.max_v] * (self.lookahead_step_num - 1)
        x_[self.first_state_index_.delta:self.num_of_x_]             = [0.0] * (self.lookahead_step_num - 1)

        # penalty on states
        for i in range(self.lookahead_step_num):
            cte = (x[self.first_state_index_.px + i] - self.end_point[0])**2 + (x[self.first_state_index_.py + i] - self.end_point[1])**2
            cost += w_cte * cte
        # penalty on inputs
        for i in range(self.lookahead_step_num - 2):
            dv     = x[self.first_state_index_.v + i + 1] - x[self.first_state_index_.v + i]
            ddelta = x[self.first_state_index_.delta + i + 1] - x[self.first_state_index_.delta + i]
            cost += w_dv*(dv**2) + w_dv*(ddelta**2)

        # define lowerbound and upperbound of x
        x_lowerbound_ = [-1e10] * self.num_of_x_
        x_upperbound_ = [1e10] * self.num_of_x_
        for i in range(self.first_state_index_.v, self.first_state_index_.delta):
            x_lowerbound_[i] = -self.max_v
            x_upperbound_[i] = self.max_v

        for i in range(self.first_state_index_.delta, self.num_of_x_):
            x_lowerbound_[i] = -0.1
            x_upperbound_[i] = 0.1

        # define lowerbound and upperbound of g constraints
        g_lowerbound_ = [0] * self.num_of_g_
        g_upperbound_ = [0] * self.num_of_g_

        g_lowerbound_[self.first_state_index_.px]    = state[0]
        g_lowerbound_[self.first_state_index_.py]    = state[1]
        g_lowerbound_[self.first_state_index_.theta] = state[2]

        g_upperbound_[self.first_state_index_.px]    = state[0]
        g_upperbound_[self.first_state_index_.py]    = state[1]
        g_upperbound_[self.first_state_index_.theta] = state[2]

        for i in range(1 + self.first_state_index_.theta + 1 * self.lookahead_step_num, self.num_of_g_):
            g_lowerbound_[i] = self.safety_r**2
            g_upperbound_[i] = 1e10

        # define g constraints
        g = [None] * self.num_of_g_
        g[self.first_state_index_.px]    = x[self.first_state_index_.px]
        g[self.first_state_index_.py]    = x[self.first_state_index_.py]
        g[self.first_state_index_.theta] = x[self.first_state_index_.theta]
        for i in range(self.num_of_agent):
            g[self.first_state_index_.theta + (i + 1) * self.lookahead_step_num] = 0

        for i in range(self.lookahead_step_num - 1):
            curr_px_index    = i + self.first_state_index_.px
            curr_py_index    = i + self.first_state_index_.py
            curr_theta_index = i + self.first_state_index_.theta
            curr_v_index     = i + self.first_state_index_.v
            curr_delta_index = i + self.first_state_index_.delta

            curr_px    = x[curr_px_index]
            curr_py    = x[curr_py_index]
            curr_theta = x[curr_theta_index]
            curr_v     = x[curr_v_index]
            curr_delta = x[curr_delta_index]

            next_px    = x[1 + curr_px_index]
            next_py    = x[1 + curr_py_index]
            next_theta = x[1 + curr_theta_index]

            next_m_px    = curr_px + curr_v*self.lookahead_step_timeinterval*cos(curr_theta)
            next_m_py    = curr_py + curr_v*self.lookahead_step_timeinterval*sin(curr_theta)
            next_m_theta = curr_theta + curr_v*self.lookahead_step_timeinterval*tan(curr_delta)

            # equality constraints
            g[1 + curr_px_index]    = next_px - next_m_px
            g[1 + curr_py_index]    = next_py - next_m_py
            g[1 + curr_theta_index] = next_theta - next_m_theta

            # inequality constraints
            for j in range(self.num_of_agent):
                g[1 + curr_theta_index + (j + 1) * self.lookahead_step_num] = (
                          next_px - agent_state_pred[j][1 + i][0])**2 + (next_py - agent_state_pred[j][1 + i][1])**2

        # create the NLP
        nlp = {'x':x, 'f':cost, 'g':vertcat(*g)}

        # solver options
        opts = {}
        opts["ipopt.print_level"] = 0
        opts["print_time"] = 0
        opts["ipopt.tol"] = 0.01
        opts["ipopt.compl_inf_tol"] = 0.001
        opts["ipopt.constr_viol_tol"] = 0.01

        solver = nlpsol('solver', 'ipopt', nlp, opts)

        # solve the NLP
        res = solver(x0=x_, lbx=x_lowerbound_, ubx=x_upperbound_, lbg=g_lowerbound_, ubg=g_upperbound_)
        return res

def animate(i):

    global cur_pos, dist_to_goal, time_stamp

    if not pause:
        # get predicted future states of the agents
        agent_pos_pred = []
        for j in range(num_of_agent):
            agent_state = []
            for k in range(lookahead_step_num):
                if time_stamp + k < len(agent_pt_list[j]):
                    agent_state.append(agent_pt_list[j][time_stamp + k])
                else:
                    agent_state.append(agent_pt_list[j][len(agent_pt_list[j]) - 1])
            agent_pos_pred.append(agent_state)

        if dist_to_goal > thres:

            # convert from DM to float
            cur_pos = list(map(float, cur_pos))

            # plot robot position
            current_pos.set_data(cur_pos[0], cur_pos[1])

            # plot agent positions
            agent_pos_1.set_data(agent_pos_pred[0][0][0], agent_pos_pred[0][0][1])
            danger_x = agent_pos_pred[0][0][0] + safety_r * np.cos(theta)
            danger_y = agent_pos_pred[0][0][1] + safety_r * np.sin(theta)
            agent_danger_zone_1.set_data(danger_x, danger_y)

            agent_pos_2.set_data(agent_pos_pred[1][0][0], agent_pos_pred[1][0][1])
            danger_x = agent_pos_pred[1][0][0] + safety_r * np.cos(theta)
            danger_y = agent_pos_pred[1][0][1] + safety_r * np.sin(theta)
            agent_danger_zone_2.set_data(danger_x, danger_y)

            agent_pos_3.set_data(agent_pos_pred[2][0][0], agent_pos_pred[2][0][1])
            danger_x = agent_pos_pred[2][0][0] + safety_r * np.cos(theta)
            danger_y = agent_pos_pred[2][0][1] + safety_r * np.sin(theta)
            agent_danger_zone_3.set_data(danger_x, danger_y)

            # solve for optimal control actions
            sol       = mpc_.Solve(cur_pos, agent_pos_pred)
            v_opt     = sol['x'][3 * lookahead_step_num]
            delta_opt = sol['x'][4 * lookahead_step_num - 1]

            # simulate forward
            cur_pos[0] = cur_pos[0] + v_opt * lookahead_step_timeinterval * cos(cur_pos[2])
            cur_pos[1] = cur_pos[1] + v_opt * lookahead_step_timeinterval * sin(cur_pos[2])
            cur_pos[2] = cur_pos[2] + v_opt * lookahead_step_timeinterval * tan(delta_opt)

            dist_to_goal = sqrt((cur_pos[0] - end_point[0])**2 + (cur_pos[1] - end_point[1])**2)

            time_stamp += 1

            return current_pos, agent_pos_1, agent_danger_zone_1, agent_pos_2, agent_danger_zone_2, agent_pos_3, agent_danger_zone_3

if __name__ == '__main__':

    # MPC parameters
    lookahead_step_num = 5
    lookahead_step_timeinterval = 0.1

    # start point and end point of ego robot
    start_point = [0.0, 0.0, 0.0]
    end_point = [1, 0]

    # agent velocity
    agent_vel = 0.1

    # start point and end point of agents
    agent_start = [[0.5, -0.5], [0.6,-0.2], [0.8, -0.5]]
    agent_end   = [[0.5, 0.5], [0.6, 0.7], [1, 1]]

    num_of_agent = len(agent_start)

    # threshold of safety
    safety_r = 0.1

    # max vx, vy
    max_v = 0.3

    agent_pt_list = []

    # calculate the coordinates of agents
    for i in range(num_of_agent):
        start_pos = agent_start[i]
        end_pos   = agent_end[i]

        if i == 0:
            agent_vel = 0.3
        elif i == 1:
            agent_vel = 0.5
        else:
            agent_vel = 0.2

        dist = sqrt((start_pos[0] - end_pos[0])**2 + (start_pos[1] - end_pos[1])**2)
        num_of_points = dist / (agent_vel * lookahead_step_timeinterval) + 1
        xs = np.linspace(start_pos[0], end_pos[0], num_of_points)
        ys = np.linspace(start_pos[1], end_pos[1], num_of_points)
        point_list = []
        for agent_x, agent_y in zip(xs, ys):
            point_list.append([agent_x, agent_y])
        agent_pt_list.append(point_list)

    mpc_ = MPC(end_point=end_point,
               num_of_agent=num_of_agent,
               safety_r=safety_r,
               max_v=max_v,
               lookahead_step_num=lookahead_step_num,
               lookahead_step_timeinterval=lookahead_step_timeinterval)

    thres = 1e-2
    pause = False

    cur_pos = copy.deepcopy(start_point)
    dist_to_goal = sqrt((cur_pos[0] - end_point[0])**2 + (cur_pos[1] - end_point[1])**2)
    time_stamp = 0

    # create animation
    fig = plt.figure(figsize=(7, 7))
    fig.canvas.mpl_connect('button_press_event', onClick)

    theta = np.arange(0, 2*np.pi, 0.01)

    plt.plot(start_point[0], start_point[1], 'o', label='start point')
    plt.plot(end_point[0], end_point[1], 'o', label='target point')

    current_pos, = plt.plot([],[], ls='None', color='k', marker='o', label='current position')
    agent_pos_1, = plt.plot([],[], ls='None', color='r', marker='o', label='agent')
    agent_danger_zone_1, = plt.plot([],[], 'r--', label='danger zone')
    agent_pos_2, = plt.plot([],[], ls='None', color='r', marker='o')
    agent_danger_zone_2, = plt.plot([],[], 'r--')
    agent_pos_3, = plt.plot([],[], ls='None', color='r', marker='o')
    agent_danger_zone_3, = plt.plot([],[], 'r--')

    plt.legend(loc='upper left')
    plt.axis([-0.1, 1.1, -0.1, 1.1])
    plt.axis('equal')
    plt.grid()

    ani = animation.FuncAnimation(fig, animate, frames=100, interval=200)

    plt.show()
