# DAZZY: Mini Robot Dog

DAZZY is a lightweight, low-cost, 3D-printed quadruped robot designed for research, education, and simulation-to-reality (Sim2Real) experimentation. It features rigid, straight legs (no knees) actuated by PTK-7465 servos and is controlled via ROS 2 Jazzy and Gazebo Harmonic.

## 🚀 Features

- **Rigid-Leg Locomotion**: DAZZY walks using a specialized "Shuffle Trot" gait. Because the legs have no knees, traditional large sinusoidal strides cause the robot to pole-vault. The shuffle gait uses an ultra-low amplitude (0.15 rad) and high frequency (2.0 Hz) to glide the robot smoothly across flat surfaces.
- **Dynamic Posture Stabilization**: The robot is naturally rear-heavy due to the placement of the battery. To prevent it from pitching up and toppling backward over time, DAZZY uses a **Low-Pass Filtered IMU Pitch Controller**. It reads data from an MPU6050-style IMU, filters out the high-frequency vibrations of walking, and applies a gentle posture bias to physically shift the center of mass forward, allowing for indefinite, stable walking.
- **High-Fidelity Simulation**: The Gazebo Harmonic simulation environment has been painstakingly configured to match the real world, virtually eliminating the Sim2Real gap.

## 🔬 Simulation Realism (Sim2Real)

The `Mini_Dog_URDF_description` package contains a deeply accurate digital twin of the physical DAZZY robot:

- **DART Physics Engine**: Uses the DART engine running a Dantzig LCP solver at 1000 Hz for mathematically rigorous articulated joint simulation.
- **Material Friction Matching**: Simulates the exact grip of the 3D printed parts. The floor is modeled as smooth vitrified tile (μ=0.55), while the robot's feet are modeled as Shore A 95 TPU rubber (μ=0.95) with a highly damped contact model (`kp=1e5`, `kd=15.0`) to simulate the 10% gyroid infill cushion pads.
- **Actuator Dynamics**: The PTK-7465 servos are simulated as position controllers running at 333 Hz (matching their physical PWM frequency), with strict physical enforcement of their 0.568 N·m stall torque and 14.95 rad/s no-load velocity limits.
- **Sensor Noise**: The simulated MPU6050 IMU injects realistic Gaussian noise and random walk drift into the accelerometer and gyroscope, ensuring that the control logic is robust enough for cheap, real-world sensors.

## 🛠️ Requirements

- **OS**: Ubuntu 24.04 (Noble)
- **ROS 2**: Jazzy Jalisco
- **Simulation**: Gazebo Harmonic (Gz Sim 8)
- **Python**: 3.12+

## 🏃‍♂️ Quick Start

### 1. Build the Workspace
```bash
# Navigate to the workspace root
cd ~/Desktop/Mini\ Robot\ Dog/DAZZY

# Source ROS 2 and build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 2. Launch the Simulation
This will boot up Gazebo Harmonic, load the `lab_world.sdf` physics environment, spawn DAZZY, and start the `ros2_control` hardware interfaces.
```bash
source install/setup.bash
ros2 launch Mini_Dog_URDF_description gazebo.launch.py
```

### 3. Run the Locomotion Controller
In a separate terminal, run the open-loop trot node. This node generates the Shuffle Trot gait and runs the IMU Pitch Controller to keep DAZZY balanced.
```bash
source install/setup.bash
ros2 run Mini_Dog_URDF_description walk_forward_node
```

## 🧠 Architecture Overview

- `urdf/Mini_Dog_URDF.xacro`: The core physical and visual definition of the robot.
- `urdf/Mini_Dog_URDF.gazebo`: Defines all physically-based rendering (PBR) materials, friction coefficients, and IMU sensor noise.
- `config/lab_world.sdf`: The DART physics engine configuration and lighting.
- `kinematics/walk_forward.py`: The Python ROS 2 node containing the gait generator and stabilization control loops.
