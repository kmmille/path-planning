import matplotlib.pyplot as plt
import csv

lens = []
times = []

with open('names2.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        if float(row[-1]) != 0 and row[-4]!='':
            lens.append(float(row[-2])-1)
            times.append(float(row[-4]))

lens1 = []
times1 = []

with open('names4.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        if float(row[-1]) != 0 and row[-4]!='':
            lens1.append(float(row[-2])-1)
            times1.append(float(row[-4]))

lens2 = []
times2 = []

with open('names6.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        if float(row[-1]) != 0 and row[-4]!='':
            lens2.append(float(row[-2])-1)
            times2.append(float(row[-4]))

plt.plot(lens, times, ls = 'None', marker = 'o', label = '75 pedestrians')
plt.plot(lens1, times1, ls = 'None', marker = 'o', label = '4 pedestrians')
plt.plot(lens2, times2, ls = 'None', marker = 'o', label = '104 pedestrians')

plt.grid()
plt.legend()
plt.xlabel('Number of line segments')
plt.ylabel('Computation time (s)')
plt.show()
