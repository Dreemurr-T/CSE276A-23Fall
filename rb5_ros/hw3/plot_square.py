import matplotlib.pyplot as plt
import numpy as np

xs = np.loadtxt('x2.txt')
ys = np.loadtxt('y2.txt')

# xi = [1.090258, 1.12593775, 0.600366, 0.07231, -0.5113, -0.56347, -0.06858, 0.45359]
# yi = [0.115953, 0.51314875, 1.09925, 1.1533, 0.6172, 0.04925, -0.5225, -0.54934]

xi = np.array([1.09025826,1.21557628, 0.6711063, 0.1890479, -0.3920370, -0.44833976, 0.008715, 0.6124853])
yi = np.array([0.3615953,0.99129002, 1.455038, 1.4853000, 0.94984482, 0.4291871, -0.26443, -0.26443])

yerr = [0.1117, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11]
xerr = [0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11, 0.11]

plt.scatter(xs, ys, c='black')
# plt.scatter(xi, yi, c='r')
plt.errorbar(xi, yi, yerr=yerr, xerr=xerr, fmt='o', ecolor='r', )
plt.show()