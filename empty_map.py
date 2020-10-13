import numpy as np

def map(pos):
    A = np.array([[-1, 0], [1, 0], [0, -1], [0, 1]])
    A_time = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0], [0, 0, -1], [0, 0, 1]])

    x = pos[0]
    y = pos[1]
    b_theta = np.array([[-(x-1)],[x+1],[-(y-1)],[y+1]])
    theta = [A, b_theta]

    b_goal = np.array([[-133], [138], [-20], [-25]])
    goal = [A, b_goal]

    b1 = np.array([[-113], [113+19], [-9], [9 + 21], [0], [1e6]])
    b2 = np.array([[-113], [113+19], [22], [-22 + 21], [0], [1e6]])
    b3 = np.array([[-141], [141+19], [-9], [9 + 21], [0], [1e6]])
    b4 = np.array([[-141], [141+19], [22], [-22 + 21], [0], [1e6]])
    obs = [[A, b1], [A, b2], [A, b3], [A, b4]]

    return theta,goal, obs
