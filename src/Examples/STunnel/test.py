import numpy as np
import matplotlib.pyplot as plt



a = [4, 1, 0,4, 1, -4,4, 0, -4,4, 0, 0,5, 0, -4,5, 0, 0,5, 1, 0,5, 1, -4,1, 0, 0,1, 1, 0,5, 1, 1,5, 0, 1,1, 0, 1,1, 1, 1,1, 1, 5,1, 0, 5,0, 0, 0,0, 1, 0,0, 0, 5,0, 1, 5,-1, 2.5, -4,-1, -2.5, -4,-1, -2.5, 5,-1, 2.5, 5,6, 2.5, -4,6, 2.5, 5,6, -2.5, -4,6, -2.5, 5]

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

a = np.reshape(a, (int(len(a)/3), 3))
x = a[:,0]
y = a[:,1]
z = a[:,2]

ax.scatter(x,y,z, marker='o')
plt.show()

