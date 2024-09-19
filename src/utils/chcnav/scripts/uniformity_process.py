#!/usr/bin/python3.6

import sys
import numpy as np
import matplotlib.pyplot as plt

print('open file:' + sys.argv[1])
data = np.loadtxt(sys.argv[1]).tolist()

time_old = int(data[0] * 1000)
time_gap = []
for i in range(1, len(data)):
    time_gap.append(int(data[i] * 1000) - time_old)
    time_old = int(data[i] * 1000)

# 线粗细
linewidth = 0.1

hline_index = []
hline_index.append(round(sum(time_gap) / len(time_gap), 2))
hline_index.append(max(time_gap))
hline_index.append(min(time_gap))

print('average:', hline_index[0])
print('max:', hline_index[1])
print('min:', hline_index[2])

x_max = len(time_gap) + 1
y_max = hline_index[1] * 6 / 5

if (len(sys.argv) < 3):
    plt.title(sys.argv[1])
else:
    plt.title(sys.argv[2])
plt.xlim(0, x_max)
plt.ylim(0, y_max)

plt.plot(range(1, len(time_gap) + 1), time_gap, linewidth=linewidth)

average_label = 'average=' + str(hline_index[0])
plt.hlines(hline_index[0], 0, x_max, color='r',
           linestyles='dashed', label=average_label)

max_label = 'max=' + str(hline_index[1])
plt.hlines(hline_index[1], 0, x_max, color='b',
           linestyles='dashed', label=max_label)

min_label = 'min=' + str(hline_index[2])
plt.hlines(hline_index[2], 0, x_max, color='y',
           linestyles='dashed', label=min_label)

plt.legend()

plt.savefig(sys.argv[1] + '.jpg')
