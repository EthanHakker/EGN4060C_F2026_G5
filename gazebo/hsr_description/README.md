# HSR Search and Rescue Robot Simulation

**EGN 4060C - Introduction to Robotics | Fall 2026 | Group 5**

## Project Overview

This project involves developing a mobile search and rescue robot capable of navigating hazardous environments, mapping its surroundings, and identifying people using computer vision.

The robot was designed in SOLIDWORKS and imported into Gazebo for simulation and testing. The simulation uses ROS 2 to communicate with the robot's sensors and control its movement.

The ultimate goal is to integrate SLAM, A* path planning, and CNN-based human detection into a functioning autonomous search system.

## Team Members

- Ethan Hacker
- Alfred Woodny Pierre
- Jahdiel Dones
- Yasmany Rodriguez

## Robot Design

The robot features:

- Four-wheel skid-steer drive system
- 2D LiDAR sensor for environmental mapping
- Rotating RGB camera for human detection
- Front-facing distance sensor for obstacle detection
- Odometry for tracking robot movement
- Aluminum chassis designed in SOLIDWORKS

The robot's CAD assembly was exported into meshes and converted into a ROS 2-compatible simulation package.

## Software Requirements

The simulation was developed and tested using:

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3
- ROS 2 Gazebo integration (`ros_gz`)
- ROS 2 Colcon build tools

Windows users can run Ubuntu through WSL2 without replacing Windows or dual-booting.

## Installation

### 1. Install ROS 2 Jazzy

Follow the official installation instructions:

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Install the required packages after configuring the ROS 2 repository:

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
sudo apt install -y ros-jazzy-ros-gz
```

### 2. Clone the repository

```bash
cd ~
git clone https://github.com/EthanHakker/EGN4060C_F2026_G5.git
```

### 3. Create the ROS 2 workspace

```bash
mkdir -p ~/hsr_ws/src
cp -r ~/EGN4060C_F2026_G5/gazebo/hsr_description ~/hsr_ws/src/
```

### 4. Install dependencies and build

```bash
cd ~/hsr_ws
source /opt/ros/jazzy/setup.bash
```

If rosdep has not been initialized:

```bash
sudo rosdep init
```

Then run:

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Original Test Environment

The original simulation contains an 8 m × 8 m test arena with walls, a doorway, debris, and obstacles.

Launch it using:

```bash
ros2 launch hsr_description sim.launch.py
```

This launches the original `hsr_test_world.sdf` environment.

### Human Detection Environment

A second world was created specifically for testing human detection.

**World file:** `hsr_human_detection_world.sdf`

It contains:

- The original test arena
- Two stationary human models
- Interior walls and obstacles
- The HSR robot

The human models are named `victim_1` and `victim_2`.

Their positions are:

| Model | X (m) | Y (m) |
|---|---|---|
| victim_1 | 3.0 | 3.0 |
| victim_2 | -2.5 | -2.5 |

Launch the human-detection world:

```bash
ros2 launch hsr_description sim.launch.py \
world:=$HOME/hsr_ws/src/hsr_description/worlds/hsr_human_detection_world.sdf
```

The human models are loaded from Gazebo Fuel. An internet connection may be required during the first launch.

Both human models have been successfully loaded and visually verified in Gazebo.

## ROS 2 Topics

The following interfaces have been identified and tested in the simulation.

| Topic | Message Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR measurements |
| `/front_range/scan` | `sensor_msgs/msg/LaserScan` | Front distance sensor |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear and angular velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Robot position and orientation estimates |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB camera images |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration information |
| `/camera_pan/cmd_pos` | `std_msgs/msg/Float64` | Camera pan control |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot joint states |

To view the available topics, open another Ubuntu terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hsr_ws/install/setup.bash
ros2 topic list -t
```

## Sensor Testing

### LiDAR

```bash
ros2 topic echo /scan --once
```

### Front Distance Sensor

```bash
ros2 topic echo /front_range/scan --once
```

### Odometry

```bash
ros2 topic echo /odom --once
```

All three topics have been verified to publish data successfully.

Odometry provides the robot's estimated position and orientation. SLAM-based localization will be integrated separately to support mapping and navigation.

## Manual Robot Control

Install the keyboard controller:

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

With Gazebo running, open another terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/hsr_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -r cmd_vel:=/cmd_vel
```

Basic controls:

| Key | Action |
|---|---|
| `i` | Move forward |
| `,` | Move backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop |

The robot has been successfully driven through the Gazebo environment using ROS 2 velocity commands.

## Camera Testing

The robot includes a rotating RGB camera.

To view the camera feed, install the image viewer:

```bash
sudo apt install -y ros-jazzy-rqt-image-view
```

Then launch it:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/camera/image_raw
```

This displays the robot's live camera feed.

### Camera Rotation

The camera can be rotated using ROS 2 commands.

Neutral position:

```bash
ros2 topic pub --once /camera_pan/cmd_pos \
std_msgs/msg/Float64 "{data: 0.0}"
```

Rotate approximately 90 degrees:

```bash
ros2 topic pub --once /camera_pan/cmd_pos \
std_msgs/msg/Float64 "{data: 1.57}"
```

Rotate approximately -90 degrees:

```bash
ros2 topic pub --once /camera_pan/cmd_pos \
std_msgs/msg/Float64 "{data: -1.57}"
```

### Human Detection Camera Test

The human-detection world has been tested successfully.

The robot was manually driven toward one of the simulated human models while monitoring `/camera/image_raw` using `rqt_image_view`.

The camera successfully captured a human model, including its head, torso, arms, and legs.

This confirms that the simulation can provide camera images containing human targets for future neural network testing.

**Note:** Human detection by a CNN has not yet been implemented or validated. The current test confirms camera visibility, not automated recognition.

## Mapping and Autonomous Navigation

The simulation provides the sensor and control interfaces needed for integrating the team's mapping and navigation algorithms.

Relevant topics include:

- `/scan` for LiDAR measurements
- `/odom` for odometry-based pose estimates
- `/cmd_vel` for robot movement commands
- `/tf` and `/tf_static` for coordinate transforms

Planned software integration includes:

1. SLAM-based mapping and localization
2. A* path planning
3. Autonomous robot movement
4. Obstacle avoidance
5. CNN-based human detection
6. Reporting detected human locations

These components will be developed and integrated with the Gazebo simulation as the project progresses.

## Current Development Status

| Component | Status |
|---|---|
| SOLIDWORKS robot model | Complete |
| Robot Gazebo simulation | Working |
| Four-wheel movement | Verified |
| LiDAR sensor output | Verified |
| Front distance sensor output | Verified |
| Odometry output | Verified |
| ROS 2 velocity control | Verified |
| Camera image output | Verified |
| Camera rotation | Verified |
| Human models in Gazebo | Complete |
| Human visibility through robot camera | Verified |
| SLAM integration | Pending |
| Autonomous navigation | Pending |
| CNN human detection | Pending |
| Full system integration | Pending |

## Notes

- The original test world is preserved separately from the human-detection world.
- The human-detection world uses stationary human models for initial testing.
- The robot's original CAD geometry does not need to be modified to run the human-detection environment.
- Gazebo Fuel resources may require an internet connection during initial loading.
- All verified simulation tests were performed using ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04 through WSL2.

## Project Goal

The final objective is to demonstrate a simulated search and rescue robot capable of autonomously exploring an environment, mapping its surroundings, navigating around obstacles, detecting people through its camera, and reporting their locations.
