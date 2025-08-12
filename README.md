<img width="700" height="600" alt="Figure_3 1" src="https://github.com/user-attachments/assets/d15c1d00-79e4-4825-a994-9f1d2884b544" /># 🎯 Missile Tracking and Interception using Kalman Filter

This project demonstrates the simulation of tracking a ballistic missile in 2D space using noisy sensor data and a Kalman Filter for state estimation. It shows how accurate prediction enables timely interception.

---

## 📊 Features

- Simulated ballistic missile trajectory
- Noisy radar-like measurements
- Real-time Kalman Filter estimation
- Visualization of true path, noisy measurements, and filtered path

---

## ▶ How to Run

```bash
pip install -r requirements.txt
python src/kalman_simulation.py
```

Output will be saved to `plots/kalman_result.png`

---

## 📁 Project Structure

```
kalman-missile-sim/
├── src/
│   └── kalman_simulation.py
├── plots/
│   └── kalman_result.png
├── requirements.txt
└── README.md
```

---

## 📷 Output Example

![Missile Interception](plots/KalmanResult.png)

This figure shows:
- 🔵 Blue line → True missile path  
- 🔴 Red dots → Noisy radar measurements  
- 🟢 Green line → Estimated path using Kalman Filter

### 🎞️ Dynamic Interception Simulation

This animation shows a full missile interception scenario:

- A ballistic missile is launched at 45° and follows a realistic trajectory under gravity.
- The interceptor is launched late from 200 km away with a slightly higher speed.
- It has no prior knowledge of the missile’s path — only noisy (X, Y) measurements.
- A 2D Kalman Filter estimates the missile’s position in real-time.
- The interceptor adjusts its direction based on predictions until it reaches within 100 meters of the target.
- An explosion is triggered upon interception, and the simulation stops.

![Missile Interception Simulation](plots/missile_interception_success.gif)

### 3D Visualization Extension

While the core calculations in this project were originally implemented in 2D,  
the same Extended Kalman Filter (EKF) framework can be directly extended to 3D space without modifying the underlying estimation logic.  
To illustrate this capability, three different 3D simulation scenarios were executed.  
Each scenario uses different initial launch parameters (speed, azimuth, elevation) and/or measurement noise levels,  
demonstrating the EKF’s robustness under varying conditions.

For each scenario, the following outputs are provided:
1. **Estimation errors** for position (X, Y, Z) and velocity (Vx, Vy, Vz) over time.  
2. **True trajectory** of the ballistic missile in 3D space.  
3. **Estimated trajectory** produced by the EKF.

---

#### Scenario 1 – Moderate speed, medium noise
**Estimation Errors:**  
![EKF Errors 1](plots/GR/Figure_1.1.png)  
**True Trajectory:**  
![True Trajectory 1](plots/GR/Figure_1.2.png)  
**EKF Estimated Trajectory:**  
![EKF Trajectory 1](plots/GR/Figure_1.3.png)  

#### Scenario 2 – Higher launch speed, lower measurement noise
**Estimation Errors:**  
![EKF Errors 2](plots/GR/Figure_2.1.png)  
**True Trajectory:**  
![True Trajectory 2](plots/GR/Figure_2.2.png)  
**EKF Estimated Trajectory:**  
![EKF Trajectory 2](plots/GR/Figure_2.3.png)  

#### Scenario 3 – Lower launch speed, higher measurement noise
**Estimation Errors:**  
![EKF Errors 3](plots/GR/Figure_3.1.png)  
**True Trajectory:**  
![True Trajectory 3](plots/GR/Figure_3.2.png)  
**EKF Estimated Trajectory:**  
![EKF Trajectory 3](plots/GR/Figure_3.3.png)  

---

💡 **Note:**  
The differences between the scenarios highlight how the EKF adapts to varying launch conditions and sensor accuracy,  
maintaining convergence and accurate state estimation in all tested cases.





Created by Ohad Marhozi, 2025  
Simulation of stochastic control in defense systems.
