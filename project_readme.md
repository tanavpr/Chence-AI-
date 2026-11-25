# Decentralized LiDAR-Based Leader-Follower Control for TurtleBot3 Burger

**Course:** CS F407 Artificial Intelligence (AY 2026--27, Semester I)  
**Institution:** BITS Pilani, K.K. Birla Goa Campus  
**Repository:** [https://github.com/tanavpr/Chence-AI-](https://github.com/tanavpr/Chence-AI-)

---

## Overview

This repository contains the ROS 2 Humble implementation of a decentralized leader-follower platooning system for the **TurtleBot3 Burger**. Unlike centralized multi-robot systems that rely on overhead cameras or inter-robot Wi-Fi communication, each follower robot operates solely using its onboard $360^\circ$ LDS-01 2D LiDAR (`/scan`) and publishes velocity commands (`/cmd_vel`) directly to its wheels.

### Control Pipeline
1. **Front-Cone Spatial Filter:** Removes invalid returns (`0.0`, `inf`, `NaN`), filters distances to $[0.12\,\text{m}, 1.0\,\text{m}]$, and restricts the field of view to a $\pm 35^\circ$ forward cone to reject nearby walls.
2. **Cluster Centroid Estimation:** Identifies the closest valid LiDAR return $d_{\min}$ and averages all points within $d_{\min} + 0.08\,\text{m}$ to estimate the leader's distance $d_t$ and bearing $\theta_t$. Triggers a zero-velocity failsafe if no target is detected in the cone.
3. **Dual-PD Velocity Control:** Regulates linear velocity $v$ and angular velocity $\omega$ to maintain a target separation of $d_{\text{ref}} = 0.30\,\text{m}$ with a $\pm 0.02\,\text{m}$ stationary deadband, clamped to TurtleBot3 hardware limits ($|v| \le 0.22\,\text{m/s}$, $|\omega| \le 1.5\,\text{rad/s}$):
   $$v = K_{p,d} e_d + K_{d,d} \frac{\Delta e_d}{\Delta t}, \qquad \omega = K_{p,\theta} e_\theta + K_{d,\theta} \frac{\Delta e_\theta}{\Delta t}$$

---

## Team Members

| Name | ID | Role & Contributions |
| :--- | :--- | :--- |
| **Tanav Prasad** | `2024A4PS0427G` | Leader-follower node design, dual-PD controller implementation |
| **Aamil Finoj** | `2024ADPS0873` | Front-cone LiDAR spatial filter, cluster centroid extraction |
| **Manav Lahoti** | `2024A7PS0543G` | TurtleBot3 hardware bringup, QoS alignment, Raspberry Pi deployment |
| **Atharv Rathore** | `2024ADPS0877G` | Gazebo simulation setup, PID gain tuning, testing |

---

## Prerequisites

### Hardware
* $2\times$ **TurtleBot3 Burger** mobile robots (Raspberry Pi 4, OpenCR controller board, $360^\circ$ LDS-01 LiDAR)
* Shared $2.4\,\text{GHz}$ Wi-Fi network or mobile hotspot (for SSH monitoring and leader teleoperation)

### Software
* **OS:** Ubuntu 22.04 LTS (on both Raspberry Pis and host PC)
* **Middleware:** ROS 2 Humble Hawksbill
* **Python:** Python 3.10+ with `rclpy`, `sensor_msgs`, `geometry_msgs`, and `numpy`
* **ROS 2 Packages:** `turtlebot3_bringup`, `turtlebot3_teleop`

---

## Repository Structure

```text
Chence-AI-/
├── follower.py    # Decentralized LiDAR front-cone filter & dual-PD follower node
└── README.md      # Setup, deployment, and reproduction instructions
```

---

## Installation & Deployment

1. **Clone the repository:**
   ```bash
   git clone https://github.com/tanavpr/Chence-AI-.git
   cd Chence-AI-
   ```

2. **Transfer `follower.py` to the Follower Raspberry Pi** (if cloned on a host PC):
   ```bash
   scp follower.py <FOLLOWER_USER>@<FOLLOWER_IP>:~/
   ```

3. **Source ROS 2 Humble** on both Raspberry Pis:
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=burger
   ```

---

## Running on Physical Hardware

To prevent `/scan` and `/cmd_vel` topic collisions on the shared Wi-Fi network, run the **Leader** and **Follower** on separate `ROS_DOMAIN_ID` values.

### 1. Start the Leader Robot (`ROS_DOMAIN_ID=1`)
Open two SSH terminals connected to the **Leader** TurtleBot3:

* **Terminal 1 (Hardware Bringup):**
  ```bash
  export ROS_DOMAIN_ID=1
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_bringup robot.launch.py
  ```
  *(Wait until `Calibration End` appears.)*

* **Terminal 2 (Keyboard Teleoperation):**
  ```bash
  export ROS_DOMAIN_ID=1
  export TURTLEBOT3_MODEL=burger
  ros2 run turtlebot3_teleop teleop_keyboard
  ```

### 2. Start the Follower Robot (`ROS_DOMAIN_ID=2`)
Place the **Follower** TurtleBot3 approximately $0.30\,\text{m}$ to $0.60\,\text{m}$ directly behind the Leader, then open two SSH terminals connected to the **Follower** Pi:

* **Terminal 3 (Hardware Bringup):**
  ```bash
  export ROS_DOMAIN_ID=2
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_bringup robot.launch.py
  ```
  *(Wait until `Calibration End` appears.)*

* **Terminal 4 (Leader-Follower Node):**
  ```bash
  export ROS_DOMAIN_ID=2
  python3 ~/follower.py
  ```

Drive the Leader forward (`w`) and through wide turns (`a`/`d`) in Terminal 2; the Follower automatically tracks the Leader and maintains a $0.30\,\text{m}$ separation.

---

## Reproducing the Reported Results

### Controller Parameters
The results reported in our paper compare two controller configurations over **3-minute indoor floor trials** (straightaways and wide turns) at an $\approx 5\,\text{Hz}$ LiDAR scan rate using the `sensor_data` QoS profile:

| Parameter | Baseline (P-Only) | Ours (Filtered Dual-PD) |
| :--- | :---: | :---: |
| Target Separation ($d_{\text{ref}}$) | $0.30\,\text{m}$ | $0.30\,\text{m}$ |
| Linear Gains ($K_{p,d}, K_{d,d}$) | $0.8,\; 0.0$ | $0.8,\; 0.1$ |
| Angular Gains ($K_{p,\theta}, K_{d,\theta}$) | $1.5,\; 0.0$ | $1.5,\; 0.1$ |
| Front-Cone Angle / Range | $\pm 35^\circ$ / $[0.12, 1.0]\,\text{m}$ | $\pm 35^\circ$ / $[0.12, 1.0]\,\text{m}$ |
| Cluster Window / Deadband | None / $\pm 0.02\,\text{m}$ | $d_{\min} + 0.08\,\text{m}$ / $\pm 0.02\,\text{m}$ |

### Steps to Reproduce
1. **Filtered Dual-PD Trial:** Run `python3 ~/follower.py` with the default gains ($K_{p,d} = 0.8$, $K_{d,d} = 0.1$, $K_{p,\theta} = 1.5$, $K_{d,\theta} = 0.1$) for 3 minutes while driving the leader along straight paths and wide turns. Record the instantaneous separation error $|d_t - 0.30|$ and frames where a valid target cluster is retained.
2. **Baseline (P-Only) Trial:** Set $K_{d,d} = 0.0$ and $K_{d,\theta} = 0.0$ (and disable cluster averaging by using raw $d_{\min}$) in `follower.py`, then repeat the same 3-minute trajectory.
3. **Compare Metrics:** Compute mean gap error ($\text{m}$) and target-lock retention ($\%$) across the logged scan callbacks.

### Summary of Experimental Results

| Method | Mean Gap Error (m) $\downarrow$ | Target-Lock Retention (%) $\uparrow$ |
| :--- | :---: | :---: |
| Baseline (P-Only) | `0.064` | `84.0%` |
| **Ours (Filtered Dual-PD)** | **`0.018`** | **`98.5%`** |