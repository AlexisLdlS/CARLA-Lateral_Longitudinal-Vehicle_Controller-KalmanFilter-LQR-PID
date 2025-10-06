# 🚗 CARLA Lateral & Longitudinal Vehicle Controller (PID + EKF + LQR)

This repository implements a **modular 2D vehicle control architecture** for the [CARLA Simulator](https://carla.org/), integrating:
- **PID controller** for longitudinal (speed) control  
- **Extended Kalman Filter (EKF)** for yaw-rate and velocity estimation  
- **LQR controller** for lateral (steering) control  

It is designed for **autonomous path following** along predefined waypoints, offering a complete and well-documented example of modern control design for self-driving applications.

---

## 🧩 System Overview

The controller is divided into three main modules:

     ┌────────────────────────────┐
     │         Longitudinal       │
     │           Control           │
     │   PID → Throttle / Brake    │
     └──────────────┬──────────────┘
                    │
                    ▼
     ┌────────────────────────────┐
     │   EKF State Estimation     │
     │   Yaw, Yaw rate, Velocity  │
     └──────────────┬──────────────┘
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
- Uses a simplified **bicycle model** for motion prediction  
- Includes process noise `Q` and measurement noise `R` tuning for balance between model and sensor trust  

✅ **Lateral Control (LQR)**  
- Minimizes lateral deviation (`e_y`) and heading error (`e_ψ`)  
- Dynamically linearized system model  
- Weight matrices `Q` and `R` customizable for response trade-offs  
- Output limited to ±1.22 rad steering angle  

✅ **Fully documented code**  
Each controller component (PID, EKF, LQR) is explained with inline comments for learning and research purposes.

---

## 📂 Repository Structure

CARLA-Lateral_Longitudinal-Vehicle_Controller-KalmanFilter-LQR-PID/
├── Controller2D.py              # Main control implementation (PID + EKF + LQR)
├── cutils.py                    # Utility class for persistent variables
├── racetrack_waypoints.txt      # Example path for testing
├── demo/
│   ├── run_controller.py        # Example integration script with CARLA
│   └── plot_results.py          # (Optional) visualization of performance
├── requirements.txt             # Python dependencies
├── .gitignore
└── README.md

---

## 🖥️ Installation & Setup

### 1. Install CARLA
Download the simulator from [CARLA official site](https://carla.org/).  
Make sure the server is running:
`CarlaUE4.exe -quality-level=Low`

### 2. Clone this repository
`git clone https://github.com/AlexisLdlS/CARLA-Lateral_Longitudinal-Vehicle_Controller-KalmanFilter-LQR-PID.git`
`cd CARLA-Lateral_Longitudinal-Vehicle_Controller-KalmanFilter-LQR-PID`

### 3. Install dependencies
`pip install -r requirements.txt`

Main dependencies:
- numpy  
- scipy  
- matplotlib  

---

## 🚀 Running the Controller

1. Launch CARLA Simulator.  
2. Run the example controller:  
`python demo/run_controller.py`  
3. The vehicle will spawn at the initial pose:  
`position = (-184, 80, 2)`  
`orientation = (0, -1, 0)`  
4. The controller will follow the waypoints defined in `racetrack_waypoints.txt`.

---

## 📊 Example Output

Example console logs show the controller’s internal diagnostics:

[Frame 139] Time: 0.02 s  
[Sim] X: -183.80  Y: 80.20  Yaw: -1.5708 rad  V: 0.00 m/s  
[EKF] Yaw: -1.5708 rad  V: -1.56 m/s  YawRate: 0.0000 rad/s  
Psi_ref: -1.5741 rad  
[Err] e_y: -2.4638 m  e_psi: 0.0033 rad  
[LQR] K: [[-0.31622777 -0.71785289 -6.56667139]]  
[Actuators] Throttle: 1.00  Brake: 0.00  Steer: -0.78  
------------------------------------------------------------

These logs illustrate real-time state estimation and control law outputs.

---

## 🧠 Controller Details

### 🔸 PID (Longitudinal)
Control signal:  
`u = Kp * e + Ki * ∫e dt + Kd * de/dt`  

Includes:  
- Anti-windup logic when actuator saturates  
- Smooth transition between acceleration and braking  

### 🔸 Extended Kalman Filter (EKF)
Predicts yaw and velocity using a bicycle model:  
ψ_dot = ψ_dot  
v_dot = a  
ψ_ddot = (v / L) * tan(δ)  

Linearized using the Jacobian `F` and corrected by sensor measurements (yaw, velocity).

### 🔸 LQR (Lateral)
Minimizes cost function:  
`J = ∫ (xᵀ Q x + uᵀ R u) dt`  

with state vector:  
`x = [e_y, e_ψ, ψ_dot]ᵀ`  

and control law:  
`δ = -Kx`

---

## ⚖️ Tuning Parameters

| Parameter | Description | Default |
|------------|-------------|----------|
| `Kp, Ki, Kd` | PID gains | 3.0, 0.6, 0.1 |
| `Q` | State penalty matrix | diag([1.0, 1.0, 0.5]) |
| `R` | Control penalty | 10.0 |
| `Q`, `R` (EKF) | Noise covariances | Q = diag([0.05,0.05,0.2]), R = diag([0.02,0.05]) |

---

## 🧭 Future Improvements

- Integrate full **MPC (Model Predictive Control)** for lateral dynamics  
- Add **adaptive speed profiling** based on curvature  
- Introduce **noise simulation** for robust testing  
- Support **dynamic waypoint loading** from CSV or ROS topics  

---

## 🤝 Credits

Developed by **Alexis L. de la S.**  
Vehicle Dynamics & Control Engineer | Model-Based Development | AUTOSAR | MATLAB/Simulink  
🔗 [GitHub](https://github.com/AlexisLdlS) · [LinkedIn](https://linkedin.com/in/alexisldls)

---

## ⚠️ License

This project is distributed for educational and research purposes under the **MIT License**.  
CARLA Simulator assets are © Computer Vision Center (CVC) and are **not included** in this repository.
