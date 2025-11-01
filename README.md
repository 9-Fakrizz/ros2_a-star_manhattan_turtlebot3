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
### Key Function Descriptions

| Function | ROS Topic/Trigger | Description |
| :--- | :--- | :--- |
| `odom_callback(msg)` | `/odom` (Odometry) | **Locomotion and Path Management.** Updates the robot's global position and orientation. It converts the global position to a grid cell. It manages the path state: triggering the **initial A\* calculation**, checking if the path is **blocked or complete**, and publishing the `Twist` command to drive the robot towards the next cell in the path. |
| `lidar_callback(msg)` | `/scan` (LaserScan) | **Obstacle Detection.** Processes the raw LiDAR data. It converts detected close-range obstacles from the robot's frame to a global grid cell coordinate and adds them to the persistent `self.obstacles` set. If **new** obstacles are found, it **clears the current path** to force a replan in the next `odom_callback` cycle. |
| `a_star(start, goal, obstacles)` | Triggered in `odom_callback` | **Path Planning.** Implements the A\* search algorithm. It uses a **cost function** (1.0 for orthogonal moves, $\sqrt{2}$ for diagonal moves) and the **Manhattan distance heuristic** to efficiently find the shortest path of non-obstacle cells from the `start` to the `goal`. |
| `update_plot(frame)` | `FuncAnimation` (GUI Loop) | **Visualization.** Updates the Matplotlib plots with the robot's current position, the detected obstacle cells, the calculated A\* path, and the raw LiDAR points. |

---
## 🧭 การทำงานของ A\* ในโค้ดนี้

อัลกอริทึม A\* ในโปรเจกต์นี้ (ไฟล์หลักคือ `move_to_target.py` ภายในแพ็กเกจ `robot401`) ถูกใช้เพื่อค้นหาเส้นทางที่สั้นที่สุดจากตำแหน่งเริ่มต้นของหุ่นยนต์ไปยังตำแหน่งเป้าหมายบน **ตารางกริด (Grid Map)**

A\* จะคำนวณ **ค่าใช้จ่ายรวม (Total Estimated Cost)** ของการเดินทางผ่านเซลล์ใดๆ บนแผนที่ โดยใช้สูตร:

$$f(n) = g(n) + h(n)$$

| สัญลักษณ์ | ชื่อเรียก | ความหมายในโค้ด |
| :---: | :--- | :--- |
| $\mathbf{g(n)}$ | **Movement Cost (ต้นทุนที่ใช้ไปจริง)** | คือ **ระยะทางสะสมจริง** จากจุดเริ่มต้น (Start) มาถึงเซลล์ปัจจุบัน $n$ |
| $\mathbf{h(n)}$ | **Heuristic Cost (ต้นทุนโดยประมาณ)** | คือ **ระยะทางโดยประมาณ** จากเซลล์ปัจจุบัน $n$ ไปยังจุดเป้าหมาย (Goal) โค้ดนี้ใช้ **Manhattan Distance** |
| $\mathbf{f(n)}$ | **Total Cost (ต้นทุนรวมที่ประมาณการ)** | ใช้ในการจัดลำดับความสำคัญของเซลล์ที่จะสำรวจ เซลล์ที่มีค่า $f(n)$ ต่ำที่สุดจะถูกเลือกก่อน |

---

## 🛠️ รายละเอียดการคำนวณ

### 1. ต้นทุนการเคลื่อนที่จริง $\mathbf{g(n)}$

ต้นทุนนี้จะสะสมเพิ่มขึ้นตามประเภทการเคลื่อนที่ในแต่ละครั้ง:

* **เคลื่อนที่ในแนวแกน (Orthogonal):** ขึ้น, ลง, ซ้าย, ขวา **(ต้นทุน $1.0$)**
* **เคลื่อนที่ในแนวทแยง (Diagonal):** **(ต้นทุน $\sqrt{2} \approx 1.414$)**

### 2. Heuristic Cost $\mathbf{h(n)}$ (Manhattan Distance)

โค้ดใช้สูตร Manhattan Distance ซึ่งคำนวณโดยไม่สนการเคลื่อนที่แนวทแยง (คิดเฉพาะการเคลื่อนที่ในแนวแกน X และ Y):

$$h(n) = |x_{current} - x_{goal}| + |y_{current} - y_{goal}|$$

---

## 📝 ตัวอย่างการคำนวณรอบแรก

สมมติให้:

* **แผนที่:** ตารางกริด
* **จุดเริ่มต้น (Start):** $S = (x_S, y_S) = (1, 1)$
* **จุดเป้าหมาย (Goal):** $G = (x_G, y_G) = (5, 4)$
* **รอบเริ่มต้น:** A\* จะเริ่มจากจุด $S$ และพิจารณาเซลล์รอบๆ (เพื่อนบ้าน) ทั้งหมด 8 ทิศทาง

### ขั้นที่ 1: คำนวณค่าที่จุดเริ่มต้น $(1, 1)$

* **$\mathbf{g(S)}$:** $0$ (ยังไม่ได้เคลื่อนที่)
* **$\mathbf{h(S)}$:** $|5-1| + |4-1| = 4 + 3 = 7$
* **$\mathbf{f(S)}$:** $0 + 7 = 7$

### ขั้นที่ 2: พิจารณาเซลล์เพื่อนบ้านในรอบแรก

A\* จะคำนวณค่า $f(n)$ สำหรับเพื่อนบ้านทั้งหมด (ในที่นี้คือ 8 ทิศทาง) โดยที่ **$g(S)$ ของเพื่อนบ้านทุกตัวเริ่มต้นจาก $0$** และเพิ่มต้นทุนการเดินทาง **จาก $S$ ไปยังเพื่อนบ้าน**

| เพื่อนบ้าน $\mathbf{n}$ | ตำแหน่ง $(x, y)$ | ต้นทุนการเคลื่อนที่ $\mathbf{g(n)}$ (จากจุดเริ่มต้น) | Heuristic $\mathbf{h(n)}$ (Manhattan Distance) | ต้นทุนรวม $\mathbf{f(n)} = g(n) + h(n)$ |
| :---: | :---: | :---: | :---: | :---: |
| $\mathbf{NE}$ (ตะวันออกเฉียงเหนือ) | $(2, 2)$ | $\sqrt{2} \approx 1.414$ | $5$ | $\mathbf{6.414}$ **(ถูกเลือก)** |
| $\mathbf{N}$ (เหนือ) | $(1, 2)$ | $1.0$ | $6$ | $\mathbf{7.0}$ |
| $\mathbf{E}$ (ตะวันออก) | $(2, 1)$ | $1.0$ | $6$ | $\mathbf{7.0}$ |
| $\mathbf{NW}$ (ตะวันตกเฉียงเหนือ) | $(0, 2)$ | $\sqrt{2} \approx 1.414$ | $7$ | $\mathbf{8.414}$ |
| $\mathbf{SE}$ (ตะวันออกเฉียงใต้) | $(2, 0)$ | $\sqrt{2} \approx 1.414$ | $7$ | $\mathbf{8.414}$ |
| $\mathbf{S}$ (ใต้) | $(1, 0)$ | $1.0$ | $8$ | $\mathbf{9.0}$ |
| $\mathbf{W}$ (ตะวันตก) | $(0, 1)$ | $1.0$ | $8$ | $\mathbf{9.0}$ |
| $\mathbf{SW}$ (ตะวันตกเฉียงใต้) | $(0, 0)$ | $\sqrt{2} \approx 1.414$ | $9$ | $\mathbf{10.414}$ |

---

### สรุปผลลัพธ์รอบแรก

* **A\* จะเลือกเซลล์ $(2, 2)$** เพราะมีค่า **$f(n)$ ต่ำที่สุด** ($\mathbf{6.414}$).
* ในรอบถัดไป A\* จะสำรวจเพื่อนบ้านของเซลล์ $(2, 2)$ และคำนวณค่า $g(n)$ ใหม่ โดยเริ่มจากค่า **$g(2, 2) = 1.414$**

การใช้ตารางนี้ช่วยให้เห็นการตัดสินใจของอัลกอริทึม A\* ซึ่งเลือกที่จะเดินไปในทิศทางที่รวม
### ขั้นที่ 3: เลือกเซลล์เพื่อสำรวจต่อ (รอบถัดไป)

A\* จะเลือกเซลล์ที่มีค่า $f(n)$ ต่ำที่สุดจากตารางข้างบน ซึ่งคือ **$(2, 2)$** ด้วยค่า $f(n) = \mathbf{6.414}$ เพื่อสำรวจเพื่อนบ้านของมันต่อในรอบถัดไป และเริ่มคำนวณค่า $f(n)$ ใหม่ โดยที่ **$g(n)$ ของเซลล์ใหม่ จะเท่ากับ $6.414$ บวกด้วยต้นทุนการเดินทางจาก $(2, 2)$**


