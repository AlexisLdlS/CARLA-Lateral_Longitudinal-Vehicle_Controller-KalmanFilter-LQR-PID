# 🚗 CARLA Lateral & Longitudinal Vehicle Controller (PID + EKF + LQR)

This repository implements a **modular 2D vehicle control architecture** for the [CARLA Simulator](https://carla.org/), integrating:
- **PID controller with simple Anti Windup** for longitudinal (speed) control  
- **Extended Kalman Filter (EKF)** for yaw-rate and velocity estimation  
- **LQR controller** for lateral (steering) control  

It is designed for **autonomous path following** along predefined waypoints, offering a complete and well-documented example of modern control design for self-driving applications.

<p align="center">
<img width="708" height="400" alt="image" src="https://github.com/user-attachments/assets/7fd907fe-81c3-48fe-b633-e065ab6e4783" />


<img width="708" height="400" alt="image" src="https://github.com/user-attachments/assets/1d656ef1-6e11-46d6-b2b7-0996801fa476" />


<img width="708" height="400" alt="image" src="https://github.com/user-attachments/assets/6171a5a0-d1fc-49d4-ade7-e54db072abbc" />
</p>

<p align="center">
<img width="184" height="488" alt="image" src="https://github.com/user-attachments/assets/52a9cc08-1cc0-4cc1-9b5b-725524d28ea6" />
<img width="388" height="489" alt="image" src="https://github.com/user-attachments/assets/9907a60d-98f9-4687-9469-4e705b700be7" />
</p>

---

## 🧩 System Overview

The controller is divided into three main modules:
                                        
                                                  ┌─────────────────────────────┐
                                                  │    Longitudinal Control     │
                                                  │   PID → Throttle / Brake    │
                                                  └──────────────┬──────────────┘
                                                                 │
                                                                 ▼
                                                  ┌────────────────────────────┐
                                                  │   EKF State Estimation     │
                                                  │   Yaw, Yaw rate, Velocity  │
                                                  └──────────────┬─────────────┘
                                                                 │
                                                                 ▼
                                                  ┌────────────────────────────┐
                                                  │        Lateral Control     │
                                                  │   LQR → Steering Command   │
                                                  └────────────────────────────┘

This architecture ensures smooth, stable, and responsive control even under noise and uncertainty in sensor measurements.

---

## ⚙️ Features

✅ **Longitudinal Control (PID)**  
- Tracks desired velocity from the waypoint list  
- Includes **anti-windup** protection to avoid integral saturation  
- Smooth throttle–brake transition logic  

✅ **State Estimation (EKF)**  
- Estimates **yaw**, **yaw rate**, and **velocity**  
- Uses a simplified **Bicycle model** for motion prediction  
- Includes process noise `Q` and measurement noise `R` tuning for balance between model and sensor trust  

✅ **Lateral Control (LQR)**  
- Minimizes lateral deviation (`e_y`) and heading error (`e_ψ`)  
- Dynamically linearized system model  
- Weight matrices `Q` and `R` customizable for response trade-offs  
- Output limited to ±1.22 rad steering angle  

✅ **Fully documented code**  
Each controller component (PID, EKF, LQR) is explained with inline comments for learning and research purposes.

---

## 🧠 Controller Details

### 🔸 PID (Longitudinal)
<img width="330" height="152" alt="image" src="https://github.com/user-attachments/assets/229131e7-a862-4194-95f3-1e3caaa17d71" />

Control signal:  
`u = Kp * e + Ki * ∫e dt + Kd * de/dt`  

Includes:  
- Anti-windup logic when actuator saturates  
- Smooth transition between acceleration and braking  

-- 

### 🔸 Extended Kalman Filter (EKF)

The EKF estimates **yaw**, **yaw rate**, and **velocity**, combining data from the simulator (measurements) with a simplified **Bicycle model**.
     
   - From the simulator: we directly measure **yaw (ψ)** and **velocity (v)**.
   - From the model: we approximate how these states **(yaw, yaw rate, velocity)** evolve over time based on steering and acceleration.  


   The prediction step uses the **Kinematic Bicycle Model Equations** Linearized using the Jacobian `F:

   <img width="412" height="200" alt="image" src="https://github.com/user-attachments/assets/b28d2d5f-2eb5-447f-9f67-ab1a02492972" />

   `ψ_dot `

   `v_dot = a  `

   `ψ_ddot = (v / L) * tan(δ)` 

   And the correction step fuses these predictions with real sensor data.

   We use covariance matrices to balance model and measurement confidence:
   - **Q** → how much we trust the **model** dynamics.  
   - **R** → how much we trust the **sensor** readings.  

   By tuning Q and R, the filter can behave more reactively (sensor-dominant) or more predictive (model-dominant), depending on the noise level and vehicle dynamics

--  

### 🔸 LQR (Lateral)
   Minimizes cost function:  
   `J = ∫ (xᵀ Q x + uᵀ R u) dt`  

   with state vector:  
   `x = [e_y, e_ψ, ψ_dot]ᵀ`  

   where  
      - x = [ e_y,  e_ψ ,  ψ_dot ]ᵀ represents **cross-track**, **heading**, and **yaw rate errors**,  
      - and u = δ(delta) is the steering angle command.

   The LQR computes the optimal feedback gain \(K\) such that the control law is:  
   `δ = -Kx`

   This allows the controller to apply smooth and stable steering corrections.

   - **Q** controls how much we penalize deviations in lateral and heading errors.  
   - **R** limits steering aggressiveness.  

   Tuning these weights lets us trade off **precision** vs. **stability** — larger R makes the vehicle less reactive but smoother.

---

## ⚖️ Tuning Parameters

| Parameter | Description | Default |
|------------|-------------|----------|
| `Kp, Ki, Kd` | PID gains | 3.0, 0.6, 0.1 |
| `Q` | State penalty matrix | diag([1.0, 1.0, 0.5]) |
| `R` | Control penalty | 10.0 |
| `Q`, `R` (EKF) | Noise covariances | Q = diag([0.05,0.05,0.2]), R = diag([0.02,0.05]) |

---

## 🖥️ Installation & Setup

### 1. Install CARLA
Download the simulator from [CARLA official site](https://carla.org/). 

### 1. Install Python 3.5 or 3.6 (CARLA 0.8.x does not support 3.7+)
During Python installation, select “Add Python to environment variables” so the commands python and pip work globally.

### 2. Clone this repository
`git clone https://github.com/AlexisLdlS/CARLA-Lateral_Longitudinal-Vehicle_Controller-KalmanFilter-LQR-PID.git`
`cd CARLA-Lateral_Longitudinal-Vehicle_Controller-KalmanFilter-LQR-PID`

### 3. Install dependencies
`python -m pip install -r \requirements.txt --user`

Main dependencies:

- Pillow>=3.1.2
- numpy>=1.14.5
- protobuf>=3.6.0
- pygame>=1.9.4
- matplotlib>=2.2.2
- future>=0.16.0
- scipy>=0.17.0

---

## 🚀 Running the Controller

1. Open CMD in Repository Path and Launch CARLA Simulator.
`CarlaUE4.exe /Game/Maps/RaceTrack -windowed -carla-server -benchmark -fps=30`

2. The vehicle will spawn at the initial pose:  
`position = (-184, 80, 2)`  
`orientation = (0, -1, 0)`
<img width="1276" height="717" alt="image" src="https://github.com/user-attachments/assets/25d888b9-a501-4651-909a-a197fa96fe96" />

3. Change directory to "\PythonClient\Course1FinalProject" and execute the following command while CARLA is open to Run the controller: 
`python module_7.py`
  
4. The controller will follow the waypoints defined in `racetrack_waypoints.txt`.

---

## 🧭 Future Improvements

- Integrate full **MPC (Model Predictive Control)** for lateral dynamics  
- Add **adaptive speed profiling** based on curvature  
- Introduce **noise simulation** for robust testing  
- Support **dynamic waypoint loading** from CSV or ROS topics  


---

## ⚠️ License

This project is distributed for educational and research purposes under the **MIT License**.  
CARLA Simulator assets are © Computer Vision Center (CVC) and are **not included** in this repository.
