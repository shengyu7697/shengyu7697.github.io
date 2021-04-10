#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt

np1 = np.genfromtxt('data.csv', delimiter=',')
print(np1)

time = np1[:, 0]
data = np1[:, 4]
plt.plot(time, data, label='data')
plt.xlabel('time')
plt.legend()
plt.show()
