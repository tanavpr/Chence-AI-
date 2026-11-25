# Decentralized LiDAR-Based Leader–Follower Control for TurtleBot3 Burger

**Course:** CS F407 Artificial Intelligence (AY 2026–27, Semester I)  
**Institution:** BITS Pilani, K.K. Birla Goa Campus  
**Repository:** [https://github.com/tanavpr/Chence-AI-](https://github.com/tanavpr/Chence-AI-)

---

## Overview

This repository contains the ROS 2 Humble implementation of a decentralized leader–follower platooning system for the **TurtleBot3 Burger**. Unlike centralized multi-robot systems that rely on overhead cameras or inter-robot Wi-Fi communication, each follower robot operates solely using its onboard 360° LDS-01 2D LiDAR (`/scan`) and publishes velocity commands (`/cmd_vel`) directly to its wheels.

### Control Pipeline
1. **Front-Cone Spatial Filter:** Removes invalid returns (`0.0`, `inf`, `NaN`), filters distances to $[0.12\,\text{m}, 1.0\,\text{m}]$, and restricts the field of view to a $\pm 35^\circ$ forward cone to reject nearby walls.
2. **Cluster Centroid Estimation:** Identifies the closest valid LiDAR return $d_{\min}$ and averages all points within $d_{\min} + 0.08\,\text{m}$ to estimate the leader's distance $d_t$ and bearing $\theta_t$. Triggers a zero-velocity failsafe if no target is detected in the cone.
3. **Dual-PD Velocity Control:** Regulates linear velocity $v$ and angular velocity $\omega$ to maintain a target separation of $d_{\text{ref}} = 0.30\,\text{m}$ with a $\pm 0.02\,\text{m}$ stationary deadband, clamped to TurtleBot3 hardware limits ($\vert{}v\vert{} \le 0.22\,\text{m/s}$, $\vert{}\omega\vert{} \le 1.5\,\text{rad/s}$).

---

## Team Members

| Name | ID | Role & Contributions |
| :--- | :--- | :--- |
| **Tanav Prasad** | `2024A4PS0427G` | Leader–follower node design, dual-PD controller implementation |
| **Aamil Finoj** | `2024ADPS0873` | Front-cone LiDAR spatial filter, cluster centroid extraction |
| **Manav Lahoti** | `2024A7PS0543G` | TurtleBot3 hardware bringup, QoS alignment, Raspberry Pi deployment |
| **Atharv Rathore** | `2024ADPS0877G` | Gazebo simulation setup, PID gain tuning, reactive navigation |

---

## Prerequisites

### Hardware
* 2× **TurtleBot3 Burger** mobile robots (Raspberry Pi 4, OpenCR controller board, 360° LDS-01 LiDAR)
* Shared 2.4 GHz Wi-Fi network / mobile hotspot (for SSH monitoring and leader teleoperation)

### Software
* **OS:** Ubuntu 22.04 LTS (on both Raspberry Pis and host PC)
* **Middleware:** ROS 2 Humble Hawksbill
* **Python:** Python 3.10+ with `rclpy`, `sensor_msgs`, `geometry_msgs`, and `numpy`
* **Packages:** `turtlebot3_bringup`, `turtlebot3_teleop`

---

## Repository Structure

```text
Chence-AI-/
├── follower.py              # Decentralized LiDAR front-cone filter & dual-PD follower node
├── obstacle_avoidance.py    # Reactive LiDAR navigation node
└── README.md                # Setup, deployment, and reproduction instructions
