import matplotlib.pyplot as plt
import csv




oblen1 = []
timeComp1 = []
with open('names3_1.csv', 'r') as csvfile:
    spamreader1 = csv.reader(csvfile, delimiter=',')
    for row in spamreader1:
        oblen1.append(float(row[-1]))
        timeComp1.append(float(row[-3]))



oblen2 = []
timeComp2 = []
with open('names_1.csv', 'r') as csvfile:
    spamreader2 = csv.reader(csvfile, delimiter=',')
    for row in spamreader2:
        oblen2.append(float(row[-1]))
        timeComp2.append(float(row[-3]))



oblen3 = []
timeComp3 = []
with open('names3.csv', 'r') as csvfile:
    spamreader3 = csv.reader(csvfile, delimiter=',')
    for row in spamreader3:
        oblen3.append(float(row[-1]))
        timeComp3.append(float(row[-3]))



oblen4 = []
timeComp4 = []
with open('names.csv', 'r') as csvfile:
    spamreader4 = csv.reader(csvfile, delimiter=',')
    for row in spamreader4:
        oblen4.append(float(row[-1]))
        timeComp4.append(float(row[-3]))


oblen4_4 = []
timeComp4_4 = []
with open('names5.csv', 'r') as csvfile:
    spamreader4 = csv.reader(csvfile, delimiter=',')
    for row in spamreader4:
        oblen4_4.append(float(row[-1]))
        timeComp4_4.append(float(row[-3]))



oblen5_1 = []
timeComp5_1 = []
with open('names5_1.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        oblen5_1.append(float(row[-1]))
        timeComp5_1.append(float(row[-3]))

oblen5 = []
timeComp5 = []
with open('names2.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        if float(row[-1]) != 0 and row[-4]!='':
            oblen5.append(float(row[-1]))
            timeComp5.append(float(row[-4]))



# plt.plot(oblen5, timeComp5, ls = 'None', marker='o')

oblen6 = []
timeComp6 = []
with open('names4.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        # print(row[-4] == '')
        if float(row[-1]) != 0 and row[-4]!='':
            oblen6.append(float(row[-1]))
            timeComp6.append(float(row[-4]))
#

oblen7 = []
timeComp7 = []
with open('names6.csv', 'r') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=',')
    for row in spamreader:
        # print(row[-4] == '')
        if float(row[-1]) != 0 and row[-4]!='':
            oblen7.append(float(row[-1]))
            timeComp7.append(float(row[-4]))

# plt.plot(oblen6, timeComp6, ls = 'None', marker='o')
plt.plot(oblen1, timeComp1, ls = 'None', color='r', marker='o', markersize = 3, label = 'MPC Lookahead time=1')
plt.plot(oblen2, timeComp2, ls = 'None', color='r', marker='o', markersize = 3)
plt.plot(oblen5_1, timeComp5_1, ls = 'None', color='r', marker='o', markersize = 3)

plt.plot(oblen3, timeComp3, ls = 'None', color='b', marker='o', markersize = 3, label = 'MPC Lookahead time=4')
plt.plot(oblen4, timeComp4, ls = 'None', color='b', marker='o', markersize = 3)
plt.plot(oblen4_4, timeComp4_4, ls = 'None', color='b', marker='o', markersize = 3)

plt.plot(oblen5, timeComp5, ls = 'None', color='g', marker='o', markersize = 3, label = 'FACTEST')
plt.plot(oblen6, timeComp6, ls = 'None', color='g', marker='o', markersize = 3)
plt.plot(oblen7, timeComp7, ls = 'None', color='g', marker='o', markersize = 3)

plt.legend()
plt.grid()
plt.xlabel('Number of obstacles')
plt.ylabel('Raw computation time')
plt.show()
