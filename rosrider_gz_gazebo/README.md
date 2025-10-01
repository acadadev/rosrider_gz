## Simulation Environments with rosrider_gz_gazebo

### 1. Absolute Odometry and TF (world_empty)

This environment uses an **empty world** where **odometry** and the **`tf` (transform)** data are derived directly from the **Gazebo (gz sim) world**. This means the robot's pose is **absolute** and perfectly accurate relative to the simulation origin.

```
ros2 launch rosrider_gz_bringup world_empty.launch.py launch_rviz:=True
```

---

### 2. EKF with Slippery Conditions (world_ekf)

This setup simulates a **slippery world** and relies on an **Extended Kalman Filter (EKF)** for pose estimation. It **only uses the `/odom` topic** from the simulation; the simulation **does not** broadcast `/tf` data. The EKF node actively **listens to `/odom` and `/imu/data`** to calculate and **broadcast the necessary `/tf`** (transform) information.

```
ros2 launch rosrider_gz_bringup world_ekf.launch.py launch_rviz:=True
```

---

### 3. Maze Simulation (Absolute Odometry) (world_maze)

A simulation featuring a **maze environment**. Like the `empty` world, it uses the **absolute coordinates** from the Gazebo world to generate the robot's **odometry and `tf`**, simplifying navigation and localization tasks.

```
ros2 launch rosrider_gz_bringup world_maze.launch.py launch_rviz:=True
```

---

### 4. Willow Mini World (world_willow)

This is a **simplified version of the Willow world**, specifically scaled and configured for **small robots**.
The ground is slippery, and the robot relies solely on encoder odometry and encoder pose for its localization, without using an EKF filter.

```
ros2 launch rosrider_gz_bringup world_willow.launch.py launch_rviz:=True
```

---

### 5. Moon Simulation with Explorer R2 (world_moon)

Experience a simulation of a **Moon environment** featuring the **Explorer R2 Robot**. 🚀

```
ros2 launch rosrider_gz_bringup world_moon.launch.py launch_rviz:=True
```

---

### 6. Cappadocia Simulation with Husky (world_cappa)

Explore a **Cappadocia-themed world** with the **Husky Robot**.

```
ros2 launch rosrider_gz_bringup world_cappa.launch.py launch_rviz:=True
```