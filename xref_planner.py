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

        for i in range(3):
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
        model.addConstr(xlist[i+1][2] - xlist[i][2] >= 5)
    return None

def add_avoidance_constraints(model, xlist, obs, max_segs, bloat_factors):
    for seg_num in range(max_segs):
        bloat_factor = 2
        
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
        x_min, y_min = -goal[1][0], -goal[1][2] 
        x_max, y_max = goal[1][1], goal[1][3]
        x_c = (x_min + x_max)/2
        y_c = (y_min + y_max)/2
        xf = [x_c, y_c]
        
        for i in range(num_segs+1):
            xnew = m.addVars(dims+1)
            tem_obj = (xnew[0] - x_c)*(xnew[0] - x_c) + (xnew[1] - y_c)*(xnew[1] - y_c) + xnew[2]
            obj += tem_obj
            xlist.append(xnew)
         
        m.setObjective(obj, GRB.MINIMIZE)
        m.setParam(GRB.Param.OutputFlag, 0)

        add_initial_constraints(m, xlist[0], Theta)
        add_final_constraints(m, xlist[-1], goal)
        add_avoidance_constraints(m, xlist, obs, num_segs, bloat_list)
        add_time_constraints(m, xlist)
        add_length_constraints(m, xlist, 10)

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

# if __name__ == '__main__':
    # obs, Theta, goal, search_area = problem()
    # A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    # b0 = np.array([[-0.5], [1], [-0.5], [1]])
    # Theta = (A, b0)

    # bf = np.array([[-5], [6], [-5], [6]])
    # goal = (A, bf)

    # At = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])
    # b1 = np.array([[-3], [4], [0], [1], [0], [1]], dtype = np.float64)

    # obs = [(At, b1)]

    # bloat_list = [0.1 for i in range(10)]

    # try:
        # xref,yref,tref = find_xref(Theta, goal, obs, 10, 1, bloat_list)
        # print(xref, yref, tref)
        # print(tref[-1] - tref[-2])

    # except:
        # print('no values!')
