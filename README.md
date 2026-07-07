# Real-Time Crowd Navigation with SORT + MPC

A ROS2-based social navigation system that enables a robot to navigate smoothly through dynamic crowds of moving people in real time. The system combines SORT (Simple Online and Realtime Tracking) for pedestrian perception with Model Predictive Control (MPC) for trajectory optimization — allowing the robot to anticipate future pedestrian positions and plan smooth, collision-free paths proactively.

---

## Demo

> Phase 1 — RViz2 Simulation (current)

The robot (blue cube) navigates toward a user-clicked goal while dynamically replanning its path (green markers) around 7 randomly spawned pedestrians (orange spheres). Pedestrian velocities are estimated in real time using Kalman filtering.

https://github.com/user-attachments/assets/ecb38c4a-9e08-4e67-a38a-4422618dff96

> Phase 2 — Real Perception (coming soon)

iPhone video → YOLOv8n + Depth Anything v2 on Jetson → SORT tracking → MPC navigation. Split-screen demo showing raw camera feed alongside the RViz2 top-down map.

---

## Architecture

```
pedestrian_sim (C++)          iPhone + RealSense (Phase 2)
       ↓                               ↓
/pedestrian_detections          YOLOv8n + Depth Anything v2
  (PoseArray)                          ↓
       ↓                        /pedestrian_detections
sort_tracker (Python)                  ↓
  Kalman Filter per track      sort_tracker (Python)
       ↓                         Kalman Filter per track
/tracked_states                        ↓
  (TrackedPedestrianArray)      /tracked_states
       ↓                               ↓
mpc_planner (Python)           mpc_planner (Python)
  CasADi IPOPT optimizer         CasADi IPOPT optimizer
       ↓                               ↓
/mpc_path + /robot_marker      /mpc_path + /robot_marker
       ↓                               ↓
     RViz2                           RViz2
```

---

## How It Works

### SORT — Perception Brain

**Simple Online and Realtime Tracking** assigns a persistent ID to each pedestrian and estimates their current velocity vector using a Kalman filter. The filter maintains a state vector `[px, py, vx, vy]` per track, running a predict-update cycle at 10Hz. Velocity is never directly measured — it is inferred by watching how position changes over time.

- Subscribes to `/pedestrian_detections` (`PoseArray`) — raw positions from sim or real perception
- Publishes `/tracked_states` (`TrackedPedestrianArray`) — position + velocity per tracked pedestrian

### MPC — Control Brain

**Model Predictive Control** is a receding-horizon optimizer. Every 300ms it asks: *given where each pedestrian is heading, what is the smoothest path from my current position to the goal?*

It uses **CasADi with the IPOPT solver** to minimize a cost function over a 10-step, 3-second prediction horizon:

| Cost Term | Weight | Purpose |
|---|---|---|
| Distance to goal | 1.0 | Pull robot toward destination |
| Path smoothness (jerk) | 3.0 | Prevent jerky movement |
| Pedestrian proximity | 2.0 | Repel robot from predicted crowd positions |

The optimizer solves for the full future trajectory, executes only the first step (receding horizon), then replans. This allows the robot to smoothly weave around people before collisions happen — not after.

- Subscribes to `/tracked_states` and `/clicked_point` (RViz2 goal)
- Publishes `/mpc_path` and `/robot_marker` for RViz2 visualization

### Custom ROS2 Messages

```
TrackedPedestrian.msg
  int32 id
  float64 px
  float64 py
  float64 vx
  float64 vy

TrackedPedestrianArray.msg
  TrackedPedestrian[] pedestrian
```

---

## Tech Stack

| Component | Technology |
|---|---|
| Framework | ROS2 Jazzy |
| Language | C++ (sim node) + Python (tracker, planner) |
| MPC Solver | CasADi + IPOPT |
| Kalman Filter | filterpy |
| Visualization | RViz2 |
| Detection (Phase 2) | YOLOv8n (Ultralytics) |
| Depth Estimation (Phase 2) | Depth Anything v2 (TensorRT FP16) |
| Hardware (Phase 2) | NVIDIA Jetson |

---

## Setup

### Prerequisites

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-desktop

# Python dependencies
pip install filterpy casadi numpy --break-system-packages
```

### Clone and Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/allenabraham106/sort-mpc-nav.git
cd ~/ros2_ws
colcon build --packages-select sort_mpc_nav
source install/setup.bash
```

### Run

```bash
ros2 launch sort_mpc_nav sim.launch.py
```

Once RViz2 opens:
1. Add `/pedestrian_markers` → MarkerArray
2. Add `/mpc_path` → MarkerArray
3. Add `/robot_marker` → Marker
4. Click **Publish Point** in the toolbar and click anywhere on the grid to set a goal
5. Watch the robot path replan in real time around moving pedestrians

---

## Roadmap

- [x] Phase 1 — 2D simulation with dynamic pedestrian spawning
- [x] SORT tracker with Kalman filter velocity estimation
- [x] MPC trajectory optimization with CasADi
- [x] Dynamic pedestrian count (no hardcoded limits)
- [x] Custom ROS2 message types
- [ ] Phase 2 — YOLOv8n real-time detection on NVIDIA Jetson
- [ ] Depth Anything v2 monocular depth estimation (TensorRT FP16)
- [ ] Split-screen demo video (iPhone footage + RViz2 map)
- [ ] Social Force Model for improved pedestrian prediction
- [ ] LSTM trajectory predictor (learning-based prediction)

---

## About

Built during co-op at the University of Waterloo Robotics Team. Motivated by humanoid robotics and the challenge of making robots navigate safely alongside humans in unstructured environments.

**Allen Abraham** — Applied Mathematics (Controls & Communications), University of Waterloo
[GitHub](https://github.com/allenabraham106)