# 🎮 Simulation Packages

| Package                                                                                              | Description                                                      |
|------------------------------------------------------------------------------------------------------|------------------------------------------------------------------|
| [rosrider_gz_bringup](https://github.com/acadadev/rosrider_gz/tree/main/rosrider_gz_bringup)         | Launch files and high level utilities                            |
| [rosrider_gz_description](https://github.com/acadadev/rosrider_gz/tree/main/rosrider_gz_description) | The SDF description of the simulated system and any other assets |
| [rosrider_gz_gazebo](https://github.com/acadadev/rosrider_gz/tree/main/rosrider_gz_gazebo)           | Gazebo specific code and configurations                          |
| [rosrider_gz_app](https://github.com/acadadev/rosrider/tree/main/rosrider_gz_app)                    | ROS specific code and configurations                             |

## 🛠️ Instructions for Simulation Setup

This guide will help you set up the **ROSRider simulation environment.** To setup the software in your computer, enter the following commands:

```commandline
mkdir -p ~/rosrider_ws/src
cd ~/rosrider_ws/src
git clone https://github.com/acadadev/rosrider
git clone https://github.com/acadadev/rosrider_gz
git clone https://github.com/acadadev/subt_gz
git clone https://github.com/acadadev/cappa_gz
cd ..
colcon build
source install/setup.bash
```

The `rosrider_gz` packages provide the following **simulation environments:**

### 📏 Absolute Odometry and TF

This environment uses an **empty world** where **odometry** and the **`tf` (transform)** data are derived directly from the **Gazebo (gz sim) world**. This means the robot's pose is **absolute** and perfectly accurate relative to the simulation origin.

```
ros2 launch rosrider_gz_bringup world_empty.launch.py launch_rviz:=True
```

![Gazebo Simulation with Absolute Odometry](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_empty_world_simulation.png)

### 🌀 EKF with Slippery Conditions

This setup simulates a **slippery world** and relies on an **Extended Kalman Filter (EKF)** for pose estimation.
It **only uses the `/odom` topic** from the simulation; the simulation **does not** broadcast `/tf` data.
The EKF node actively **listens to `/odom` and `/imu/data`** to calculate and **broadcast the necessary `/tf`** (transform) information.

```
ros2 launch rosrider_gz_bringup world_ekf.launch.py launch_rviz:=True
```

![EKF odometry visualization](https://docs.acada.dev/rosrider_doc/images/rosrider/rviz_odometry_ekf.png)

### 🔳 Maze Simulation

A simulation featuring a **maze environment**. Like the `empty` world, it uses the **absolute coordinates** from the Gazebo world to generate the robot's **odometry and `tf`**, simplifying navigation and localization tasks.

```
ros2 launch rosrider_gz_bringup world_maze.launch.py launch_rviz:=True
```

![Gazebo Maze Simulation](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_maze_simulation.png)


### 🏢 Willow World

This is a **simplified version of the Willow world**, specifically scaled and configured for **small robots**.
The ground is slippery, and the robot relies solely on encoder odometry and encoder pose for its localization, without using an EKF filter.

```
ros2 launch rosrider_gz_bringup world_willow.launch.py launch_rviz:=True
```

![Gazebo Simulation Willow World](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_willow_world_simulation.png)

### 🌀 Willow World with EKF

This is a **simplified version of the Willow world**, specifically scaled and configured for **small robots**.  

The environment features a **slippery ground**, which degrades the accuracy of the robot's raw encoder odometry. 
To compensate, this world relies on an `Extended Kalman Filter (EKF)` for robust pose estimation. 
The EKF node actively fuses the degraded `/odom` topic (the sole odometry source from the simulation) 
with the `/imu/data` (Inertial Measurement Unit) to **calculate and broadcast** the necessary `/tf` (transform) information,
significantly improving localization accuracy over the simple odometry alone.

```
ros2 launch rosrider_gz_bringup world_willow_ekf.launch.py launch_rviz:=True
```

![Navigating Willow WWorld with EKF](https://docs.acada.dev/rosrider_doc/images/rosrider/nav_local_global_map.png)


### 🌕 Moon Simulation with Explorer R2

Experience a simulation of a **Moon environment** featuring the **Explorer R2 Robot**. 🚀

```
ros2 launch rosrider_gz_bringup world_moon.launch.py launch_rviz:=True
```

![Gazebo Simulation Moon Rover](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_moon_simulation.png)

### ⛰️ Cappadocia Simulation with Husky

Explore a **Cappadocia-themed world** with the **Husky Robot**.

```
ros2 launch rosrider_gz_bringup world_cappa.launch.py launch_rviz:=True
```

![Gazebo Simulation Cappadocia](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_cappa_simulation.png)

### 📖 Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)

