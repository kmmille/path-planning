import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
from sensor import *
from threading import Thread
from control_utils import *
from scipy.integrate import odeint
from empty_map import map
from xref_planner import *

filename = 'intersection_03_traj_ped.csv'
pedNum = 10
time_stamp = 0
time_step = 64e-3
cur_pos = [122,2,0,0,0]
stop = 0
d_sense = 10
agent_paths = createPedPath(filename, pedNum)
lookahead_step_num = 100
ts = 0
theta = []
goal = []
obs = []


def animate(i):
    global cur_pos, time_step, time_stamp, agent_paths, stop
    global waypoints, ts

    # plot car
    current_pos.set_data(cur_pos[0], cur_pos[1])
    x_bound = [cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
               cur_pos[0] + 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
               cur_pos[0] - 2*cos(cur_pos[2]) + 1.5*sin(cur_pos[2]),
               cur_pos[0] - 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2]),
               cur_pos[0] + 2*cos(cur_pos[2]) - 1.5*sin(cur_pos[2])]
    y_bound = [cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
               cur_pos[1] + 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
               cur_pos[1] - 2*sin(cur_pos[2]) - 1.5*cos(cur_pos[2]),
               cur_pos[1] - 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),
               cur_pos[1] + 2*sin(cur_pos[2]) + 1.5*cos(cur_pos[2]),]
    car_outline.set_data(x_bound, y_bound)

    # plot_pedestrians
    agent_pos_x = []
    agent_pos_y = []

    for j in range(pedNum):
        agent_pos_x.append(agent_paths[j][time_stamp][0])
        agent_pos_y.append(agent_paths[j][time_stamp][1])

    agent_pos.set_data(agent_pos_x, agent_pos_y)

    if not stop:
        wps = set_waypoints(waypoints)
        t = [time_stamp*time_step, (time_stamp+1)*time_step]
        q = run_model(cur_pos, t, wps)
        v = q[-2]
        omega = q[-1]


        cur_pos[0] = cur_pos[0] + v*cos(cur_pos[2])*time_step
        cur_pos[1] = cur_pos[1] + v*sin(cur_pos[2])*time_step
        cur_pos[2] = cur_pos[2] + omega*time_step

        time_stamp += 1

        if sqrt((cur_pos[0] - wps[2])**2 + (cur_pos[1] - wps[3])**2) < 2:
            waypoints.pop(0)
            if len(waypoints) < 2:
                stop = 1

    return current_pos, car_outline, agent_pos

def get_controller():
    global cur_pos, time_step, time_stamp
    global theta,goal,obs
    ts = 0
    while 1:
        if time_step*time_stamp - ts > 1:
            ts = time_step*time_stamp
            print('replan')
            print(len(obs))
    return None

def sense_env():
    global cur_pos, pedNum, agent_paths, time_step, time_stamp, lookahead_step_num, d_sense
    global theta, goal, obs

    while 1:
        theta, goal, obs = map(cur_pos)
        ped_preds = ped_rects(cur_pos, pedNum, agent_paths, time_step, time_stamp, lookahead_step_num, d_sense)
        obs.extend(ped_preds)
    return None

t1 = Thread(target = sense_env)
t1.daemon = True
t2 = Thread(target = get_controller)
t2.daemon = True

t1.start()
# t2.start()

if __name__ == '__main__':
    # filename = 'intersection_03_traj_ped.csv'
    # pedNum = 10
    # time_stamp = 0
    # time_step = 64e-3
    # cur_pos = [122,2,0,0,0]
    # stop = 0
    # d_sense = 10


    # agent_paths = createPedPath(filename, pedNum)





    waypoints = [[122, 2, 0], [136, 5, 10], [138, 20, 20]]

    fig, ax = plt.subplots(figsize=(7, 7))

    patches = [Rectangle((113, 9), 19, 21), Rectangle((113, -22), 19, 21), Rectangle((141, -22), 19, 21), Rectangle((141, 9), 19, 21)]
    pc = PatchCollection(patches, facecolor='red')
    ax.add_collection(pc)


    current_pos, = plt.plot([], [] , ls='None', color='b', marker='o', label='car_pos')
    car_outline, = plt.plot([], [] , color='b')
    agent_pos, = plt.plot([], [] , ls='None', color='k', marker='o', label='agent_pos')

    ani = animation.FuncAnimation(fig, animate, frames=1000, interval=20)

    plt.axis([118, 145, -3, 30])
    plt.grid()
    plt.show()
