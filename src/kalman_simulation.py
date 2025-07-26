import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter

dt = 1.0
n = 50
x_true = np.linspace(0, 500, n)
y_true = 0.5 * 9.8 * (dt * np.arange(n))**2

x_meas = x_true + np.random.normal(0, 20, n)
y_meas = y_true + np.random.normal(0, 50, n)

kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.array([[1, dt, 0,  0],
                 [0,  1, 0,  0],
                 [0,  0, 1, dt],
                 [0,  0, 0,  1]])
kf.H = np.array([[1, 0, 0, 0],
                 [0, 0, 1, 0]])
kf.R *= 100
kf.P *= 500.
kf.Q *= 0.01
kf.x = np.array([0, 1, 0, 1])

xs, ys = [], []
for i in range(n):
    z = np.array([x_meas[i], y_meas[i]])
    kf.predict()
    kf.update(z)
    xs.append(kf.x[0])
    ys.append(kf.x[2])

plt.figure(figsize=(10,6))
plt.plot(x_true, y_true, label='True path', color='blue')
plt.scatter(x_meas, y_meas, label='Noisy measurements', color='red', s=15)
plt.plot(xs, ys, label='Kalman Filter estimate', color='green')
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Missile Trajectory Tracking with Kalman Filter")
plt.legend()
plt.grid()
plt.savefig("plots/kalman_result.png")
plt.show()
