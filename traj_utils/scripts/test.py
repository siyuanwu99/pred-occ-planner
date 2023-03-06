#! /usr/bin/python3

import numpy as np

A3 = np.array([      [1, -3, 3, 1],
      [0, 3, -6, 3],
      [0, 0, 3, -3],
      [0, 0, 0, 1]])

A2 = np.array([      [1, -2, 1], [0, 2, -2], [0, 0, 1]])

print(np.linalg.inv(A2))