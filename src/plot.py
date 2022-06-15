# to run: python3.8 plot.p
from re import X
import numpy as np
import matplotlib.pyplot as plt

#plt.plot([5, 10, 15, 20, 25, 30], [6.23, 11.9, 19.1, 25.1, 31.2, 37.2], 'bo--')
#x = [2/490, 2/226, 2/152, 2/112, 2/90]

x = [2/415, 2/179, 2/119, 2/90, 2/70]
y = [10, 20, 30, 40, 50]

plt.plot(x, y, 'bo--')
plt.axis([0, 0.03, 0, 60])

slope, intercept = np.polyfit(x, y, 1)
print(slope)
print(intercept)

plt.ylabel('Motor PWM')
plt.xlabel('velocity (m/sec)')
plt.show()