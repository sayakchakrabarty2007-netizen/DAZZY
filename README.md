# DAZZY: Mini Robot Dog Quadruped 🐕🤖

DAZZY is a lightweight, low-cost, 3D-printed quadruped robot platform designed for robotics research, education, and simulation-to-reality (**Sim2Real**) experimentation. Powered by **ROS 2 Jazzy** and **Gazebo Harmonic**, DAZZY features a 4-bar parallel linkage leg design, low-pass IMU posture stabilization, and a multi-gait locomotion controller capable of walking, crawling, skateboarding, spot pivoting, and executing dynamic tricks.

---

## 🚀 Key Features & Capabilities

### 🐾 1. Advanced Locomotion & Trick Suite
DAZZY includes 5 distinct gaits and dynamic postures:

* ** Shuffle Trot Walk (`W`)**: High-frequency (2.0 Hz), low-amplitude (0.15 rad) trot gait. Uses an active IMU pitch feedback loop to compensate for rear battery mass bias and prevent backward tipping.
* ** Cute Crawl Gait (`C`)**: A smooth 4-beat crawling creep sequence (FL → RR → FR → RL) with cosine-smoothed leg trajectories and built-in yaw drift compensation.
* ** Skateboard Kick & Glide (`S`)**: A synchronous 4-leg stroke gait with fast `REACH`, strong `PUSH`, and a momentum-based `SKATE` phase.
* ** Skid-Steer Pivot Turns (`R` / `L`)**: Asymmetrically drives active leg pairs while resting the passive side to perform 360° spot turns in place.
* ** Wheelie & Sit-and-Wave (`U`)**: Reclines onto rear legs into a sit pose while waving the front legs back and forth out-of-phase.
* ** Active Stand / Posture Hold (`SPACE` / `X`)**: Holds neutral 0° posture with smooth quintic blending transitions between gaits.

---

### 🧠 2. Control System & Kinematics Architecture
* **Unified Teleop Controller (`master_teleop_node`)**: Interactive non-blocking keyboard controller with state machine management across all gaits.
* **Quintic Polynomial Motion Blending**: Uses 5th-order polynomial interpolation over a 0.15s window when switching modes, eliminating sudden joint jerks or high-current servo spikes.
* **Closed-Loop IMU Pitch Stabilization**: Low-pass filters high-frequency stance impacts from the MPU6050 IMU ($0.95/0.05$ IIR filter) and applies proportional pitch corrections ($K_p = 0.3$) to maintain body leveling.
* **Calibrated Joint Sign Matrix**: Enforces symmetric leg behavior across all four quadruped quadrants:
  $$\text{Joint Directives} = [\text{FL}: -1.0, \text{RL}: -1.0, \text{FR}: 1.0, \text{RR}: 1.0]$$

---

### ⚡ 3. Split-Brain Sim-to-Real Architecture (`use_sim_mode`)
To bridge the gap between Gazebo physics and physical hardware:
* **Physical Hardware (`use_sim_mode:=False`)**: Leverages physical one-way ratchet wheels (`FootWheel_Ratchet`) mounted on the feet, allowing smooth forward rolling while mechanically blocking rearward slip.
* **Gazebo Simulation (`use_sim_mode:=True`)**: Uses dynamic `stance_bias` stance offsets during recovery strokes to tilt the chassis and lift recovering feet, perfectly mimicking stick-slip ratchet wheel dynamics without needing complex non-holonomic contact plugins.

---

### 🔬 4. Optimized High-Performance Gazebo Physics
The simulation environment in `lab_world.sdf` has been tuned for **100% Real-Time Factor (RTF)**:
* **Inertia Tensor Regularization**: All lightweight 3D-printed link inertias are clamped to $I_{xx}, I_{yy}, I_{zz} \ge 1e-6 \text{ kg}\cdot\text{m}^2$ to prevent LCP numerical solver instability.
* **Primitive Contact Geometry**: High-poly STL foot collision meshes are replaced with lightweight `<sphere radius="0.015"/>` contacts.
* **PGS Solver Tuning**: Utilizes Gazebo Harmonic's Projected Gauss-Seidel (**PGS**) solver at 1000 Hz with high contact damping (`kp=1e5`, `kd=15.0`) for fast, robust simulation of closed 4-bar linkage chains.

---

## 🎮 Keyboard Teleop Controls

Launch `master_teleop_node` to control DAZZY in real time:

| Key | Mode / Action | Description |
| :--- | :--- | :--- |
| `W` / `w` | **Trot Walk** | Forward trot gait with IMU balance & stance bias |
| `C` / `c` | **Crawl Gait** | Cute 4-beat creep gait with yaw compensation |
| `S` / `s` | **Skateboard** | Synchronous 4-leg kick & glide gait |
| `R` / `r` | **Pivot Right** | Skid-steer spot turn right |
| `L` / `l` | **Pivot Left** | Skid-steer spot turn left |
| `U` / `u` | **Wheelie** | Recline on rear legs and wave front legs |
| `SPACE` / `X` | **Stand / Stop** | Smoothly return to neutral posture (0°) |
| `Q` / `Ctrl+C` | **Quit** | Exit teleop node safely |

---

## 💻 Hardware & Software Requirements

* **Operating System**: Ubuntu 24.04 LTS (Noble Numbat)
* **ROS 2 Version**: ROS 2 Jazzy Jalisco
* **Simulator**: Gazebo Harmonic (Gz Sim 8)
* **Python Engine**: Python 3.12+
* **Dependencies**: `rclpy`, `std_msgs`, `sensor_msgs`, `ros2_control`, `gz_ros2_control`

---

## 🏃‍♂️ Quick Start Guide

### 1. Build the ROS 2 Workspace
```bash
# Navigate to workspace root
cd ~/Desktop/Mini\ Robot\ Dog/DAZZY

# Source ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Build the package
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Gazebo Simulation
Launch Gazebo Harmonic, spawn DAZZY in `lab_world.sdf`, and initialize `ros2_control`:
```bash
source install/setup.bash
ros2 launch Mini_Dog_URDF_description gazebo.launch.py
```

### 3. Launch Master Teleop Controller
In a second terminal, launch the interactive teleop node:
```bash
source install/setup.bash

# Run in Simulation Mode (default)
ros2 run Mini_Dog_URDF_description master_teleop_node --ros-args -p use_sim_mode:=true

# Or Run on Real Hardware
ros2 run Mini_Dog_URDF_description master_teleop_node --ros-args -p use_sim_mode:=false
```

---

## 📁 Package Structure

```
DAZZY/
├── Mini_Dog_URDF_description/
│   ├── config/
│   │   ├── gz_ros2_control.yaml   # Joint controllers & state broadcaster
│   │   └── lab_world.sdf          # Gazebo Harmonic world & PGS solver setup
│   ├── launch/
│   │   └── gazebo.launch.py       # ROS 2 launch file for Gazebo & controllers
│   ├── Mini_Dog_URDF_description/
│   │   └── kinematics/
│   │       ├── master_teleop.py   # Unified keyboard teleop node
│   │       ├── walk_forward.py    # IMU-balanced trot gait controller
│   │       ├── crawl_gait.py      # 4-beat crawling creep controller
│   │       ├── skateboard_sim.py  # Kick & glide skateboard gait controller
│   │       ├── pivot_turn.py      # Skid-steer spot turn controller
│   │       └── wheelie_glide.py   # Sit & wave trick controller
│   ├── urdf/
│   │   ├── Mini_Dog_URDF.xacro    # Core URDF robot geometry & joints
│   │   └── Mini_Dog_URDF.gazebo   # Gazebo materials, friction & IMU plugin
│   ├── package.xml
│   └── setup.py                   # Package scripts & entry points
└── README.md                      # Project documentation
```

---

## 📄 License & Attribution
Developed for quadruped robotics research, Sim2Real gait optimization, and ROS 2 control systems.
