import csv
from sensor import *
from math import *

agent_path09 = createPedPath('intersection_09_traj_ped.csv', 75)
agent_path07 = createPedPath('intersection_07_traj_ped.csv', 104)
agent_path02 = createPedPath('intersection_02_traj_ped.csv', 104)

filename = 'names.csv'
pos_list = []
with open('names4.csv', 'r') as csvfile:
    spamreader4 = csv.reader(csvfile, delimiter=',')
    for row in spamreader4:
        pos_list.append([float(row[0]), float(row[1])])


mindist09 = 10000
mindist07 = 10000
mindist02 = 10000
for i in range(len(pos_list)):
    # for j in range(len(agent_path09)):
    #     mindist09 = min(mindist09, sqrt((pos_list[i][0] - agent_path09[j][i][0])**2 + (pos_list[i][1] - agent_path09[j][i][1])**2))
    # for j in range(len(agent_path07)):
    #     mindist07 = min(mindist07, sqrt((pos_list[i][0] - agent_path07[j][i][0])**2 + (pos_list[i][1] - agent_path07[j][i][1])**2))
    for j in range(len(agent_path02)):
        mindist02 = min(mindist02, sqrt((pos_list[i][0] - agent_path02[j][i][0])**2 + (pos_list[i][1] - agent_path02[j][i][1])**2))

print(mindist02,mindist07,mindist09)
