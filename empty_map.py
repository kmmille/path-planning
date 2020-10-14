import numpy as np

def map(pos):
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    A_time = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])

    x = pos[0]
    y = pos[1]
    b_theta = np.array([[-(x-0.1)],[x+0.1],[-(y-0.1)],[y+0.1]])
    theta = [A, b_theta]

    b_goal = np.array([[-133], [138], [-20], [25]])
    goal = [A, b_goal]

    b1 = np.array([[-113], [113+19], [-9], [9 + 21], [0], [1e2]])
    b2 = np.array([[-113], [113+19], [22], [-22 + 21], [0], [1e2]])
    b3 = np.array([[-141], [141+19], [-9], [9 + 21], [0], [1e2]])
    b4 = np.array([[-141], [141+19], [22], [-22 + 21], [0], [1e2]])
    obs = [[A_time, b1], [A_time, b2], [A_time, b3], [A_time, b4]]

    return theta,goal, obs
