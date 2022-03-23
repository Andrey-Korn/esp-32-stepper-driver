# Graph accel/gyro data from provided json file recorded on board

import matplotlib.pyplot as plt
import numpy
import json

# grab data
filename = 'test.json'
f = open(filename)
data = json.load(f)

for i in data:
    print(i)

# plt.figure()

# plot angles


# plt.show()