````markdown
# 🐢 ROS 2 A* Pathfinding for TurtleBot3 (Manhattan Heuristic)

## 🌟 Project Overview

This project implements a **Real-Time A* Pathfinding** algorithm on a TurtleBot3 robot using ROS 2 (Robotic Operating System 2). The robot utilizes its **LiDAR** sensor to detect obstacles dynamically and update a pre-defined grid map.

The core functionality involves:
1.  Mapping the environment into a low-resolution grid.
2.  Using the **A* search algorithm** with a **Manhattan distance-based heuristic** to find an optimal, non-colliding path from the robot's current position to a pre-set goal cell. *(Note: The current implementation uses diagonal movements with corresponding costs, which generally benefits from a different heuristic like Euclidean or Chebyshev, but the core pathfinding structure is A\*).*
3.  **Dynamic Replanning**: If the LiDAR detects a *new* obstacle that blocks the current path, the robot automatically stops and recalculates a new, collision-free route.
4.  A **Matplotlib-based GUI** provides a real-time visualization of the robot's position, the grid map, detected obstacles, and the calculated path.

---

## ✨ Features and Limitations

### Features

* **Autonomous Navigation:** The TurtleBot3 can navigate autonomously to a fixed goal position within the grid.
* **Dynamic Obstacle Avoidance:** Uses LiDAR data to dynamically detect and map obstacles into the grid.
* **A* Pathfinding:** Employs the A* algorithm (with a diagonal movement cost structure) for efficient path calculation.
* **Real-Time Visualization:** A built-in GUI displays the robot's location, the persistent obstacle map, the planned path, and the local LiDAR view.
* **Path Replanning:** Automatically triggers a path recalculation when new obstacles block the current route.

### Limitations

* **Static Goal and Map Size:** The grid dimensions (`grid_width`, `grid_height`) and the target goal (`goal_cell`) are hard-coded within the Python script.
* **Persistent Obstacle Map:** Once an obstacle is detected, it remains mapped for the entire run (no "clearing" or "obstacle decay" mechanism).
* **LiDAR Range:** Obstacle detection for mapping is limited to a small range (0.15m to 0.35m) for local planning simplicity.
* **Manhattan Heuristic (Implementation Note):** While the code uses the Manhattan heuristic, the movement model allows for diagonal moves with $\sqrt{2}$ cost, meaning the heuristic is not *perfectly* consistent for this move set, though it remains admissible and functional.

---

## 🛠️ Setup and Running Instructions

These instructions assume you have a working **ROS 2 Humble (or compatible)** environment set up for the TurtleBot3.

### 1. Export ROS_DOMAIN_ID (Required for Network Communication)

The `ROS_DOMAIN_ID` variable is crucial for the robot and your control station to communicate over the ROS 2 network.

* **Find the Official Quick Start Guide:** Please follow the official guide for setting up the ROS 2 environment and networking for your specific TurtleBot3 model (e.g., TurtleBot3 Burger/Waffle Pi).
    * **Official ROS 2 TurtleBot3 Setup Guide:** [https://emanual.robotis.com/docs/en/platform/turtlebot3/pc\_setup/](https://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)
* **On both your robot and your remote PC (where you run the node), you must set the same domain ID:**

```bash
# Set a unique domain ID (e.g., 30)
export ROS_DOMAIN_ID=<YOUR_DOMAIN_ID> 
````

> **Note:** Replace `<YOUR_DOMAIN_ID>` with an integer value (0-100 recommended).

### 2\. Run TurtleBot3 Bringup

On the TurtleBot3's embedded PC (or in the Gazebo simulation environment), start the necessary ROS 2 nodes:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

*(If using simulation, use the corresponding Gazebo launch command.)*

### 3\. Setup the Workspace

On your control PC (or the same PC running the simulation/bringup if applicable):

```bash
# Create and navigate to your workspace
mkdir -p ~/turtlebot3_astar_ws/src
cd ~/turtlebot3_astar_ws/src

# Clone the repository (replace <YOUR_REPO> with the actual repo name/URL)
git clone <YOUR_REPO_URL>

# Go back to the workspace root
cd ~/turtlebot3_astar_ws

# Build the workspace
colcon build

# Source the setup files (important to make the nodes accessible)
source install/setup.bash
```

### 4\. Run the A\* Node

Ensure the **ROS\_DOMAIN\_ID** is still exported (Step 1) and your workspace is sourced (Step 3).

```bash
# Ensure ROS_DOMAIN_ID is exported again (if you opened a new terminal)
export ROS_DOMAIN_ID=<YOUR_DOMAIN_ID> 

# Source the workspace setup file again (if you opened a new terminal)
source ~/turtlebot3_astar_ws/install/setup.bash

# Run the pathfinding node (replace 'your_pkg_name' with your actual package name)
ros2 run your_pkg_name move_to_target
```

A **Matplotlib window** will pop up, showing the real-time grid map and the robot's movement.

-----

## 🔍 Code Flowchart and Explanation

The `move_to_target.py` node operates through three main ROS 2 subscription/callback loops and the A\* algorithm itself.

### Main Code Flowchart

```mermaid
graph TD
    A[Start Node move_to_target] --> B(rclpy.spin_once)
    B --> C{Odometry Callback /odom}
    B --> D{LiDAR Callback /scan}
    C --> E(Update Robot Pos/Yaw)
    C --> F(Map Pos to Grid Cell)
    C --> G{Initial Path Calculated?}
    G -- No --> H[Run A* Pathfinding]
    G -- Yes --> I{Path Blocked/Empty?}
    I -- Yes --> J[Run A* Pathfinding (Replan)]
    I -- No --> K[Follow Next Path Cell]
    H --> L(Publish cmd_vel)
    J --> L
    K --> L
    D --> M(Process Range Data)
    D --> N(Convert Local Obstacle to Global Grid Cell)
    D --> O(Update persistent self.obstacles set)
    D --> P{New Obstacle Detected?}
    P -- Yes --> Q[Clear self.path (Force Replanning in Odom)]
    P -- No --> R[Continue]

```

### Key Function Descriptions

| Function | ROS Topic/Trigger | Description |
| :--- | :--- | :--- |
| `odom_callback(msg)` | `/odom` (Odometry) | **Locomotion and Path Management.** Updates the robot's global position and orientation. It converts the global position to a grid cell. It manages the path state: triggering the **initial A\* calculation**, checking if the path is **blocked or complete**, and publishing the `Twist` command to drive the robot towards the next cell in the path. |
| `lidar_callback(msg)` | `/scan` (LaserScan) | **Obstacle Detection.** Processes the raw LiDAR data. It converts detected close-range obstacles from the robot's frame to a global grid cell coordinate and adds them to the persistent `self.obstacles` set. If **new** obstacles are found, it **clears the current path** to force a replan in the next `odom_callback` cycle. |
| `a_star(start, goal, obstacles)` | Triggered in `odom_callback` | **Path Planning.** Implements the A\* search algorithm. It uses a **cost function** (1.0 for orthogonal moves, $\sqrt{2}$ for diagonal moves) and the **Manhattan distance heuristic** to efficiently find the shortest path of non-obstacle cells from the `start` to the `goal`. |
| `update_plot(frame)` | `FuncAnimation` (GUI Loop) | **Visualization.** Updates the Matplotlib plots with the robot's current position, the detected obstacle cells, the calculated A\* path, and the raw LiDAR points. |

```
```
