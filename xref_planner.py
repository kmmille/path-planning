# Gurobi xref planner
# Written by: Kristina Miller

from __future__ import division
from gurobipy import *
from math import *
import numpy as np
import matplotlib.pyplot as plt

def add_initial_constraints(model, x, Theta):
    # Make sure to start in Theta
    A, b = Theta
    dims = len(A[0])

    # Find center of initial set
    dx = b[0][0] + b[1][0]
    dy = b[2][0] + b[3][0]

    x0 = [float(b[1][0] - float(dx/2)), float(b[3][0] - float(dy/2)), 0]

    model.addConstrs(x[i] == x0[i] for i in range(dims))
    model.addConstr(x[2] == 0)
    return None

def add_final_constraints(model, x, goal):
    # Make sure to end in goal
    A, b = goal
    dims = len(A[0])

    # Find center of goal:
    dx = b[0][0] + b[1][0]
    dy = b[2][0] + b[3][0]

    xf = [float(b[1][0] - float(dx/2)), float(b[3][0] - float(dy/2))]

    model.addConstrs(x[i] == xf[i] for i in range(dims))
    return None

def add_obstacle_constraint(model, xlist, A, b, bloat_factor, ob_num, seg_num):
    # Avoid a polytope
    edges = len(b)
    dims = 3
    # Use bigM method to avoid polytopes
    M = 1e6
    alpha = model.addVars(edges, vtype=GRB.BINARY, name="alpha_ob"+str(ob_num)+"_seg"+str(seg_num))

    x1 = xlist[seg_num]
    x2 = xlist[seg_num+1]
    for edge in range(edges):
        tem_constr1 = 0
        tem_constr2 = 0
        h_norm = np.linalg.norm(A[edge])

        for i in range(dims):
            tem_constr1 += A[edge][i]*x1[i]
            tem_constr2 += A[edge][i]*x2[i]

        tem_constr1 = -tem_constr1 + (b[edge] + h_norm*bloat_factor)
        tem_constr2 = -tem_constr2 + (b[edge] + h_norm*bloat_factor)
        model.addConstr(tem_constr1 <= M*(1 - alpha[edge]))
        model.addConstr(tem_constr2 <= M*(1 - alpha[edge]))

    model.addConstr(alpha.sum() >= 1)
    return None

def add_time_constraints(model, xlist):
    for i in range(len(xlist) - 1):
        model.addConstr(xlist[i+1][2] - xlist[i][2] >= 15)
        model.addConstr(xlist[i+1][2] - xlist[i][2] <= 100)
    return None

def add_avoidance_constraints(model, xlist, obs, max_segs, bloat_factors):
    for seg_num in range(max_segs):

        bloat_factor = 3
        # if seg_num == 0:
        #     bloat_factor = 3

        for ob_num in range(len(obs)):
            A, b = obs[ob_num]
            add_obstacle_constraint(model, xlist, A, b, bloat_factor, ob_num, seg_num)
    return None

def add_space_constraints(model, xlist, limits):
    xlim, ylim = limits
    for x in xlist:
        model.addConstr(x[0] >= xlim[0])
        model.addConstr(x[1] >= ylim[0])
        model.addConstr(x[0] <= xlim[1])
        model.addConstr(x[1] <= ylim[1])

    return None

def add_length_constraints(model, xlist, l_min):
    # Make sure each line segment is long enough
    # Use big M method
    M = 1e6
    for seg_num in range(1, len(xlist)):
        x1 = xlist[seg_num-1][0]
        y1 = xlist[seg_num-1][1]
        x2 = xlist[seg_num][0]
        y2 = xlist[seg_num][1]

        model.addConstr((x2 - x1) + (y2 - y1) - l_min <= 0)
        model.addConstr((x2 - x1) + (y2 - y1) + l_min >= 0)
    return None

def find_xref(Theta, goal, obs, max_segs, l_min, bloat_list, old_wp = None):
    # Find the entire path
    dims = len(Theta[0][0])

    for num_segs in range(1, max_segs):
        xlist = []
        m = Model("xref")
        m.setParam(GRB.Param.OutputFlag, 0)

        obj = 0
        obj2 = 0
        x_min, y_min = -goal[1][0][0], -goal[1][2][0]
        x_max, y_max = goal[1][1][0], goal[1][3][0]
        x_c = (x_min + x_max)/2
        y_c = (y_min + y_max)/2
        xf = [x_c, y_c]
        x = []
        for i in range(len(obs)):
            x1_min, y1_min = -obs[i][1][0][0], -obs[i][1][2][0]
            x1_max, y1_max = obs[i][1][1][0], obs[i][1][3][0]
            x1 = (x1_min + x1_max)/2
            y1 = (y1_min + y1_max)/2
            x.append((x1, y1))

        for i in range(num_segs+1):
            xnew = m.addVars(dims+1)
            # if i == num_segs:
            #     obj = xnew[2]
            tem_obj = (xnew[0] - x_c)*(xnew[0] - x_c) + (xnew[1] - y_c)*(xnew[1] - y_c)# xnew[2]
            # tem_obj1 = (xnew[0] - x1)* (xnew[0] - x1)+ (xnew[1] - y1)*(xnew[1] - y1)
            # tem_obj1 = 0
            # for i in range(len(obs)):
            #     tem_obj1 += (xnew[0] - x[i][0])* (xnew[0] - x[i][0])#+ (xnew[1] - y1)*(xnew[1] - y1)
            obj += tem_obj
            xlist.append(xnew)


        m.setObjective(obj, GRB.MINIMIZE)
        m.setParam(GRB.Param.OutputFlag, 0)

        add_initial_constraints(m, xlist[0], Theta)
        add_final_constraints(m, xlist[-1], goal)
        add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
        add_time_constraints(m, xlist)
        add_length_constraints(m, xlist, 5)

        m.write("test.lp")

        m.update()
        m.optimize()

        try:
            wps_list = []

            for x in xlist:
                x_pt = x[0].X
                y_pt = x[1].X
                t_pt = x[2].X

                wps_list.append([x_pt, y_pt, t_pt])

            m.dispose()
            return wps_list

        except:
            m.dispose()

if __name__ == '__main__':

    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    A_time = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])

    x = 122
    y = 4
    b_theta = np.array([[-(x-0.1)],[x+0.1],[-(y-0.1)],[y+0.1]])
    theta = [A, b_theta]

    b_goal = np.array([[-133], [138], [-20], [25]])
    goal = [A, b_goal]

    b1 = np.array([[-113], [113+19], [-9], [9 + 21], [0], [100]])
    b2 = np.array([[-113], [113+19], [22], [-22 + 21], [0], [100]])
    b3 = np.array([[-141], [141+19], [-9], [9 + 21], [0], [100]])
    b4 = np.array([[-141], [141+19], [22], [-22 + 21], [0], [100]])
    obs = [[A_time, b1], [A_time, b2], [A_time, b3], [A_time, b4]]
    # obs, Theta, goal, search_area = problem()
    # A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    # b0 = np.array([[-0.5], [1], [-0.5], [1]])
    # Theta = (A, b0)

    # bf = np.array([[-5], [6], [-5], [6]])
    # goal = (A, bf)

    # At = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])
    # b1 = np.array([[-3], [4], [0], [1], [0], [1]], dtype = np.float64)

    # obs = [(At, b1)]

    bloat_list = [0.1 for i in range(10)]

    # try:
    wps = find_xref(theta, goal, obs, 10, 1, bloat_list)
    print(wps)
    # print(tref[-1] - tref[-2])

    # except:
        # print('no values!')
