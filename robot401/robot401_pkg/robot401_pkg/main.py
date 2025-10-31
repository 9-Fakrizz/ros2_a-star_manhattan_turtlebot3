#!/usr/bin/env python3
import math
import heapq
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # LiDAR QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile=sensor_qos)

        # Grid config
        self.offset_robot_y = 1.05
        self.goal_cell = (5, 21)
        self.grid_width = 22
        self.grid_height = 10
        self.cell_size = 0.2
        self.grid_origin_x = 0.0
        self.grid_origin_y = 0.0
    

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.start_x = None
        self.start_y = None
        self.current_cell = None
        self.path = []
        self.obstacles = set()
        self.stop_flag = False
        # Flag to ensure A* is only called once at startup
        self.initial_path_calculated = False 

        # GUI setup
        self.fig, (self.ax_grid, self.ax_lidar) = plt.subplots(1,2, figsize=(10,5))
        # --- grid map
        self.robot_plot, = self.ax_grid.plot([], [], 'bo', label='Robot')
        self.goal_plot, = self.ax_grid.plot([], [], 'gx', markersize=10, label='Goal')
        self.obs_plot, = self.ax_grid.plot([], [], 'rs', label='Obstacles')
        self.path_plot, = self.ax_grid.plot([], [], 'y-', linewidth=2, label='Path')
        self.ax_grid.set_aspect('equal')
        self.ax_grid.set_xlim(-0.2, self.grid_width*self.cell_size+0.2)
        self.ax_grid.set_ylim(-0.5, self.grid_height*self.cell_size+0.5)
        self.ax_grid.set_xlabel('X (m)')
        self.ax_grid.set_ylabel('Y (m)')
        self.ax_grid.legend()
        self.ax_grid.set_title('Robot Grid Map with Obstacles')
        self.draw_grid()

        # --- lidar plot
        self.ax_lidar.set_aspect('equal')
        self.ax_lidar.set_xlim(-0.7,0.7)
        self.ax_lidar.set_ylim(-0.7,0.7)
        self.ax_lidar.set_xlabel('X (m)')
        self.ax_lidar.set_ylabel('Y (m)')
        self.ax_lidar.set_title('LiDAR local view')
        self.lidar_plot, = self.ax_lidar.plot([], [], 'r.', markersize=5)

        self.get_logger().info("MoveToTarget with A* and LiDAR GUI started.")
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=500)
        self.latest_scan = None

    # ------------------- Draw grid -------------------
    def draw_grid(self):
        for i in range(self.grid_width+1):
            x = self.grid_origin_x + i*self.cell_size
            self.ax_grid.plot([x,x],[self.grid_origin_y,self.grid_origin_y+self.grid_height*self.cell_size],'gray',linewidth=0.5)
        for j in range(self.grid_height+1):
            y = self.grid_origin_y + j*self.cell_size
            self.ax_grid.plot([self.grid_origin_x,self.grid_origin_x+self.grid_width*self.cell_size],[y,y],'gray',linewidth=0.5)

    # ------------------- GUI update -------------------
    def update_plot(self, frame):
        # obstacles
        # NOTE: (r,c) -> x,y
        obs_x = [self.grid_origin_x + c*self.cell_size + self.cell_size/2 for (r,c) in self.obstacles]
        obs_y = [self.grid_origin_y + r*self.cell_size + self.cell_size/2 for (r,c) in self.obstacles]
        self.obs_plot.set_data(obs_x, obs_y)
        # robot
        self.robot_plot.set_data([self.robot_x],[self.robot_y])
        # goal
        gx = self.grid_origin_x + self.goal_cell[1]*self.cell_size + self.cell_size/2
        gy = self.grid_origin_y + self.goal_cell[0]*self.cell_size + self.cell_size/2
        self.goal_plot.set_data([gx],[gy])
        # path
        if self.path:
            # NOTE: (r,c) -> x,y
            px = [self.grid_origin_x + c*self.cell_size + self.cell_size/2 for (r,c) in self.path]
            py = [self.grid_origin_y + r*self.cell_size + self.cell_size/2 for (r,c) in self.path]
            self.path_plot.set_data(px,py)
        else:
            self.path_plot.set_data([],[])
        # lidar
        if self.latest_scan:
            xs,ys = [],[]
            angle = self.latest_scan.angle_min
            for r in self.latest_scan.ranges:
                # Only plot points that are within a certain range for local view
                if self.latest_scan.range_min < r < self.latest_scan.range_max and 0.05 < r <= 0.6:
                    lx = r*math.cos(angle)
                    ly = r*math.sin(angle)
                    xs.append(lx)
                    ys.append(ly)
                angle += self.latest_scan.angle_increment
            self.lidar_plot.set_data(xs,ys)
        return self.robot_plot,self.obs_plot,self.goal_plot,self.path_plot,self.lidar_plot

    # ------------------- Odometry -------------------
    def odom_callback(self,msg):
        pos = msg.pose.pose.position
        self.robot_x = pos.x
        self.robot_y = pos.y + self.offset_robot_y
        q = msg.pose.pose.orientation
        siny_cosp = 2*(q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp,cosy_cosp)

        if self.start_x is None:
            self.start_x,self.start_y = self.robot_x,self.robot_y

        # map to grid
        # NOTE: Row = Y-axis, Col = X-axis
        col = int((self.robot_x - self.grid_origin_x)//self.cell_size)
        row = int((self.robot_y - self.grid_origin_y)//self.cell_size)
        col = max(0,min(col,self.grid_width-1))
        row = max(0,min(row,self.grid_height-1))

        # Track where the robot ACTUALLY is
        self.current_cell = (row, col)
        
        # Check if cell changed AND if robot is stuck in an obstacle cell
        if self.current_cell != (row,col):
             self.current_cell = (row,col)
             if self.current_cell in self.obstacles:
                 self.get_logger().warn(f"Robot in obstacle cell {self.current_cell}! Stop and replan.")
                 self.stop_flag = True # Force an immediate stop
             else:
                 self.stop_flag = False # Clear stop flag if it was set

        # **FIX 1: Calculate initial path**
        if not self.initial_path_calculated and self.current_cell:
            self.path = self.a_star(self.current_cell, self.goal_cell, self.obstacles)
            self.initial_path_calculated = True

        # **FIX 2: Path blocked check**
        # Check if the next cell in the path is now blocked or if path is empty/blocked
        path_blocked = self.path and self.path[0] in self.obstacles # Check if current path head is an obstacle
        
        # If the path is empty, we've reached the goal or got stuck/no path.
        if self.path and self.path[0] == self.goal_cell:
             self.get_logger().info("GOAL REACHED! Stopping.")
             self.path = [] # Clear path
             self.stop_flag = True # Stop movement
        elif path_blocked or not self.path:
            # Only replan if path is blocked or empty (and not at goal)
            if self.current_cell != self.goal_cell:
                self.get_logger().info("Path blocked or empty. Replanning...")
                self.path = self.a_star(self.current_cell, self.goal_cell, self.obstacles)
                if not self.path:
                    self.get_logger().warn("Could not find a path to the goal!")
                    self.stop_flag = True # Stop if no path is found
                else:
                    self.stop_flag = False
            
        
        # move robot if not stopped
        cmd = Twist()
        if self.path and not self.stop_flag:
            # We are always trying to go to the next cell after the current one
            next_cell = self.path[1] if len(self.path) > 1 else self.path[0]
            
            # Target (x,y) is the center of the next cell
            tx = self.grid_origin_x + next_cell[1]*self.cell_size + self.cell_size/2
            ty = self.grid_origin_y + next_cell[0]*self.cell_size + self.cell_size/2
            
            dx,dy = tx - self.robot_x, ty - self.robot_y
            dist = math.hypot(dx,dy)
            target_angle = math.atan2(dy,dx)
            diff = self.normalize_angle(target_angle - self.robot_yaw)
            
            # Check if close enough to the center of the current path cell to move to the next one
            if dist < 0.15:
                # If path has more than one cell, pop the current cell (which is path[0])
                if len(self.path) > 1:
                    self.path.pop(0)
                else: # Only one cell left (the goal), stop moving after this small step
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.stop_flag = True
            else:
                # Movement logic
                if abs(diff) > 0.3: # Prioritize rotation if angle is large
                    cmd.linear.x = 0.05
                    cmd.angular.z = 0.8*diff
                else:
                    cmd.linear.x = 0.1
                    cmd.angular.z = 1.5*diff
        else:
             cmd.linear.x = 0.0
             cmd.angular.z = 0.0
             
        self.publisher_.publish(cmd)

    # ------------------- LiDAR -------------------
    def lidar_callback(self, msg):
        if self.start_x is None or self.current_cell is None:
            return

        self.latest_scan = msg
        # **FIX:** Do NOT clear self.obstacles here. 
        # Instead, build a set of obstacles detected in THIS scan.
        obstacles_in_current_scan = set() 
        
        angle = msg.angle_min
        for r in msg.ranges:
            # Check for close-range obstacles only
            if msg.range_min < r < msg.range_max and 0.15 < r <= 0.35: 
                
                # 1. Convert local (r, angle) to robot-relative (lx, ly)
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                
                # 2. Convert robot-relative (lx, ly) to global (ox, oy)
                ox = self.robot_x + lx * math.cos(self.robot_yaw) - ly * math.sin(self.robot_yaw)
                oy = self.robot_y + lx * math.sin(self.robot_yaw) + ly * math.cos(self.robot_yaw)

                # 3. Convert global (ox, oy) to grid cell (row, col)
                col = int((ox - self.grid_origin_x) // self.cell_size)
                row = int((oy - self.grid_origin_y) // self.cell_size)

                # Check if the cell is within the grid boundaries
                if 0 <= row < self.grid_height and 0 <= col < self.grid_width:
                    obstacles_in_current_scan.add((row, col))
            
            angle += msg.angle_increment

        # Check if the currently detected obstacles introduce any *new* blocks
        newly_detected_obstacles = obstacles_in_current_scan - self.obstacles
        
        # Merge the newly detected obstacles into the persistent set
        self.obstacles.update(obstacles_in_current_scan) 

        # If we found at least one new obstacle not previously mapped, force a replan.
        if newly_detected_obstacles:
            self.get_logger().info(f"New obstacle(s) detected: {newly_detected_obstacles}. Forcing replan.")
            # Clearing the path forces a replan in the next odom_callback loop
            self.path = [] 
            
        # NOTE: For simplicity, we are not implementing an "obstacle decay" or "clearing" mechanism. 
        # All detected obstacles will be remembered for the entire run.
  
    # ------------------- A* -------------------
    def a_star(self,start,goal,obstacles):
        # Heuristic: Use Diagonal Distance (Manhattan distance is okay, but Diagonal/Euclidean is often better for diagonal movement)
        def h(cell): 
            # Chebyshev distance (Max of dx, dy) is often used with uniform cost diagonal moves (cost=1)
            # but Manhattan is simple and works fine here since we differentiate costs.
            return abs(cell[0]-goal[0]) + abs(cell[1]-goal[1]) 
            
        open_set = []
        # (f_score, g_score, cell)
        heapq.heappush(open_set,(h(start),0.0,start)) # Use 0.0 for float cost
        came_from = {}
        g = {start:0.0} # Use 0.0 for float cost
        
        # Define all 8 possible movements and their associated costs
        # (dr, dc, cost) -> (change in row, change in col, cost)
        movements = [
            ( 1,  0, 1.0), # Up
            (-1,  0, 1.0), # Down
            ( 0,  1, 1.0), # Right
            ( 0, -1, 1.0), # Left
            ( 1,  1, math.sqrt(2)), # Up-Right (Diagonal)
            ( 1, -1, math.sqrt(2)), # Up-Left (Diagonal)
            (-1,  1, math.sqrt(2)), # Down-Right (Diagonal)
            (-1, -1, math.sqrt(2))  # Down-Left (Diagonal)
        ]

        self.get_logger().info(f"A* (Diagonal) started from {start} to {goal} with {len(obstacles)} obstacles.")
        
        while open_set:
            _,cost,cur = heapq.heappop(open_set)
            
            if cur==goal:
                path=[]
                while cur in came_from:
                    path.append(cur)
                    cur=came_from[cur]
                path.append(start)
                path.reverse()
                self.get_logger().info(f"Diagonal path found with {len(path)} cells.")
                return path
                
            for dr, dc, move_cost in movements:
                n_row, n_col = cur[0] + dr, cur[1] + dc
                n = (n_row, n_col)
                
                # Check boundaries
                if 0<=n_row<self.grid_height and 0<=n_col<self.grid_width:
                    if n in obstacles: continue # Skip if neighbor is an obstacle
                    
                    # Calculate new g_score
                    new_g = g[cur] + move_cost 
                    
                    if n not in g or new_g < g[n]:
                        g[n] = new_g
                        # f_score = g_score + h(n)
                        heapq.heappush(open_set,(new_g+h(n),new_g,n))
                        came_from[n] = cur
                        
        self.get_logger().warn("A* failed to find a path!")
        return []
    def normalize_angle(self,a):
        while a>math.pi: a-=2*math.pi
        while a<-math.pi: a+=2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTarget()
    # It's better to show the plot after the node is initialized
    plt.show(block=False) 
    try:
        # **Note**: For ROS 2 with Matplotlib animation, this is generally a good loop structure.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # Close plot window when shutting down
        plt.close(node.fig) 

if __name__=='__main__':
    main()