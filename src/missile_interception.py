import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter

# --- Physical Constants ---
g = 9.81  # Gravity (m/s^2)

# --- Missile Parameters ---
v0_missile = 1500  # Initial missile speed (m/s)
angle_deg = 45  # Launch angle
angle_rad = np.radians(angle_deg)
v0x = v0_missile * np.cos(angle_rad)
v0y = v0_missile * np.sin(angle_rad)

# --- Time Parameters ---
T = 2 * v0y / g  # Total flight time until impact
N = 500  # Number of time steps
t = np.linspace(0, T, N)
dt = t[1] - t[0]

# --- Missile Trajectory ---
x_m = v0x * t
y_m = v0y * t - 0.5 * g * t**2
y_m = np.maximum(y_m, 0)  # Avoid negative altitudes
missile_pos = np.stack((x_m, y_m), axis=1)

# --- Interceptor Parameters ---
delay = 50  # Delay before interceptor is launched (in frames)
v_interceptor = 1600  # Interceptor speed (m/s)
x0_i, y0_i = 200000, 0  # Interceptor initial position

# --- Kalman Filter Setup ---
A = np.array([[1, 0, dt, 0],
              [0, 1, 0, dt],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])  # State transition matrix
H = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])  # Observation matrix
Q = np.eye(4) * 0.01  # Process noise covariance
R = np.eye(2) * 100**2  # Measurement noise covariance
P = np.eye(4) * 500  # Initial estimation covariance

# Initial interceptor state [x, y, vx, vy]
state = np.array([x0_i, y0_i,
                  -v_interceptor * np.cos(angle_rad),
                   v_interceptor * np.sin(angle_rad)])

interceptor_pos = []  # Store interceptor positions
estimates = []        # Store Kalman filter estimates
exploded = False      # Flag if interception occurred
explosion_frame = None  # Frame at which interception happened

# --- Simulation Loop ---
for i in range(N):
    if i < delay:
        # Interceptor has not launched yet
        interceptor_pos.append([x0_i, y0_i])
        estimates.append(state[:2])
        continue

    # Noisy missile measurement
    meas = missile_pos[i] + np.random.multivariate_normal([0, 0], R)

    # Kalman prediction with gravity
    gravity = np.array([0, 0, 0, -g * dt])
    state = A @ state + gravity
    P = A @ P @ A.T + Q

    # Kalman update
    K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R)
    state = state + K @ (meas - H @ state)
    P = (np.eye(4) - K @ H) @ P

    # Interceptor movement toward estimated position
    direction = state[:2] - interceptor_pos[-1]
    direction /= np.linalg.norm(direction)
    new_pos = interceptor_pos[-1] + v_interceptor * dt * direction
    interceptor_pos.append(new_pos)
    estimates.append(state[:2])

    # Check if interception occurred
    dist = np.linalg.norm(missile_pos[i] - new_pos)
    if not exploded and dist < 100:
        exploded = True
        explosion_frame = i
        break

# Cut simulation at interception
max_frames = explosion_frame + 1 if exploded else N
interceptor_pos = np.array(interceptor_pos)
missile_pos = missile_pos[:max_frames]
interceptor_pos = interceptor_pos[:max_frames]

# --- Plot Setup ---
fig, ax = plt.subplots(figsize=(12, 6))
ax.set_xlim(0, np.max(x_m)*1.1)
ax.set_ylim(0, np.max(y_m)*1.2)
ax.set_title("Missile Interception using Kalman Filter")
ax.set_xlabel("Distance (m)")
ax.set_ylabel("Altitude (m)")

# Lines and markers
ln_m, = ax.plot([], [], 'r-', label="Missile")
ln_i, = ax.plot([], [], 'b--', label="Interceptor")
ln_m_dot, = ax.plot([], [], 'ro')  # Missile dot
ln_i_dot, = ax.plot([], [], 'bo')  # Interceptor dot
ln_explosion, = ax.plot([], [], 'r*', markersize=20)  # Explosion marker
ax.legend()

# --- Animation Function ---
def update(i):
    ln_m.set_data(x_m[:i], y_m[:i])
    ln_i.set_data(interceptor_pos[:i, 0], interceptor_pos[:i, 1])
    ln_m_dot.set_data([x_m[i]], [y_m[i]])
    ln_i_dot.set_data([interceptor_pos[i, 0]], [interceptor_pos[i, 1]])

    # Show explosion if intercepted
    if exploded and i == explosion_frame:
        ln_explosion.set_data([x_m[i]], [y_m[i]])
    else:
        ln_explosion.set_data([], [])

    return ln_m, ln_i, ln_m_dot, ln_i_dot, ln_explosion

# --- Save Animation as GIF ---
ani = animation.FuncAnimation(fig, update, frames=max_frames, interval=20, blit=True)
ani.save("missile_interception_success.gif", writer=PillowWriter(fps=30))
