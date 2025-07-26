import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
import os

# --- Simulation Parameters ---
dt = 1.0  # time step (seconds)
n = 50    # number of time steps

# --- True missile trajectory (2D motion under gravity) ---
x_true = np.linspace(0, 500, n)  # horizontal position
y_true = 0.5 * 9.8 * (dt * np.arange(n))**2  # vertical motion under acceleration

# --- Simulated radar measurements with Gaussian noise ---
x_meas = x_true + np.random.normal(0, 20, n)
y_meas = y_true + np.random.normal(0, 50, n)

# --- Kalman Filter Setup ---
kf = KalmanFilter(dim_x=4, dim_z=2)

# State transition matrix (position and velocity in 2D)
kf.F = np.array([[1, dt, 0,  0],
                 [0,  1, 0,  0],
                 [0,  0, 1, dt],
                 [0,  0, 0,  1]])

# Measurement function (we observe x and y only)
kf.H = np.array([[1, 0, 0, 0],
                 [0, 0, 1, 0]])

# Measurement noise covariance
kf.R *= 100

# Initial state covariance (high uncertainty)
kf.P *= 500.

# Process noise covariance (small dynamics uncertainty)
kf.Q *= 0.01

# Initial state: [x, vx, y, vy]
kf.x = np.array([0, 1, 0, 1])

# --- Kalman Filter Execution ---
xs, ys = [], []  # store estimated positions

for i in range(n):
    z = np.array([x_meas[i], y_meas[i]])  # current measurement
    kf.predict()
    kf.update(z)
    xs.append(kf.x[0])  # estimated x
    ys.append(kf.x[2])  # estimated y

# --- Output Plot ---
os.makedirs("plots", exist_ok=True)  # create output folder if missing

plt.figure(figsize=(10, 6))
plt.plot(x_true, y_true, label='True trajectory', color='blue', linewidth=2)
plt.scatter(x_meas, y_meas, label='Noisy measurements', color='red', s=20)
plt.plot(xs, ys, label='Kalman Filter estimate', color='green', linewidth=2)
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Missile Tracking using Kalman Filter")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("plots/kalman_result.png", dpi=300)
plt.show()
