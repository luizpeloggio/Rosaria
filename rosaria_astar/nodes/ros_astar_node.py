#!/usr/bin/env python3
import rospy
import numpy as np
import collections
import heapq
import time
import math

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Header

def yaw_from_quaternion(q):
    """Calculates yaw angle from geometry_msgs/Quaternion"""
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class RosAstarNode:
    def __init__(self):
        rospy.init_node('rosaria_astar_node', anonymous=True)
        
        # ROS parameters
        self.allow_diagonal = rospy.get_param('~allow_diagonal', False)
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.8) # meters
        self.u_weight = rospy.get_param('~u_weight', 15.0) # potential field weight
        
        self.kv = rospy.get_param('~kv', 0.4) # linear speed gain
        self.kw = rospy.get_param('~kw', 1.2) # angular turn gain
        self.lookahead = rospy.get_param('~lookahead', 0.4) # lookahead distance
        self.goal_tol = rospy.get_param('~goal_tol', 0.15)
        
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        
        # Grid parameters (30m x 30m window)
        self.grid_size_m = 30.0
        self.grid_x_center = 10.7
        self.grid_y_center = 4.2
        
        # State variables
        self.map_msg = None
        self.grid = None
        self.potentials = None
        self.dist_grid = None
        self.resolution = None
        self.origin = None
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.odom_received = False
        
        self.planned_path = []
        self.path_follower = None
        self.navigating = False
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        # Publishers
        self.path_pub = rospy.Subscriber('/planned_path', Path, queue_size=1) # Note: we publish on this topic, using a publisher below
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        
        rospy.loginfo("rosaria_astar_node initialized.")
        rospy.loginfo(f"Subscribed to: /map, /odom, /move_base_simple/goal")
        rospy.loginfo(f"Publishing path on: /planned_path")
        rospy.loginfo(f"Publishing cmd_vel on: {self.cmd_vel_topic}")

    def map_callback(self, msg):
        self.map_msg = msg
        self.resolution = msg.info.resolution
        self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        
        width = msg.info.width
        height = msg.info.height
        
        rospy.loginfo(f"Map received: {width}x{height} cells, res: {self.resolution}m")
        
        # Convert 1D occupancy grid to 2D numpy array
        # ROS occupancy values: -1 = unknown, 0 = free, 100 = occupied
        raw_map = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Define 30m x 30m grid window
        self.grid_size_cells = int(self.grid_size_m / self.resolution) # 300 cells for 0.1m res
        self.grid_origin_x = self.grid_x_center - (self.grid_size_m / 2.0)
        self.grid_origin_y = self.grid_y_center - (self.grid_size_m / 2.0)
        
        self.col_min = int((self.grid_origin_x - self.origin[0]) / self.resolution)
        self.row_min = int((self.grid_origin_y - self.origin[1]) / self.resolution)
        
        self.grid = np.ones((self.grid_size_cells, self.grid_size_cells), dtype=np.uint8)
        
        for r in range(self.grid_size_cells):
            map_row = self.row_min + r
            if map_row < 0 or map_row >= height:
                continue
            for c in range(self.grid_size_cells):
                map_col = self.col_min + c
                if map_col < 0 or map_col >= width:
                    continue
                
                val = raw_map[map_row, map_col]
                # In ROS maps, 0 is free space. Everything else (occupied or unknown) is blocked.
                if val == 0:
                    self.grid[r, c] = 0 # Free
                else:
                    self.grid[r, c] = 1 # Occupied/Unknown
                    
        # Compute distance transform and potentials
        self.compute_distance_transform()
        self.compute_potentials()
        rospy.loginfo("Occupancy grid and potential field constructed successfully.")

    def compute_distance_transform(self):
        """Computes distance to nearest obstacle using multi-source BFS"""
        dist = np.full(self.grid.shape, 1e9, dtype=np.float32)
        q = collections.deque()
        
        for r in range(self.grid_size_cells):
            for c in range(self.grid_size_cells):
                if self.grid[r, c] == 1:
                    dist[r, c] = 0.0
                    q.append((r, c))
                    
        directions = [
            (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]
        
        while q:
            r, c = q.popleft()
            curr_dist = dist[r, c]
            for dr, dc, step_dist in directions:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.grid_size_cells and 0 <= nc < self.grid_size_cells:
                    if dist[nr, nc] > curr_dist + step_dist:
                        dist[nr, nc] = curr_dist + step_dist
                        q.append((nr, nc))
                        
        self.dist_grid = dist * self.resolution

    def compute_potentials(self):
        self.potentials = np.zeros(self.dist_grid.shape, dtype=np.float32)
        mask = self.dist_grid < self.inflation_radius
        self.potentials[mask] = self.u_weight * (1.0 - self.dist_grid[mask] / self.inflation_radius) ** 2

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def world_to_grid(self, x, y):
        c = int((x - self.grid_origin_x) / self.resolution)
        r = int((y - self.grid_origin_y) / self.resolution)
        return r, c

    def grid_to_world(self, r, c):
        x = self.grid_origin_x + (c + 0.5) * self.resolution
        y = self.grid_origin_y + (r + 0.5) * self.resolution
        return x, y

    def goal_callback(self, msg):
        if self.grid is None:
            rospy.logwarn("No map received yet! Cannot plan path.")
            return
        if not self.odom_received:
            rospy.logwarn("No odometry received yet! Cannot localize robot.")
            return
            
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        
        rospy.loginfo(f"Received goal: ({gx:.2f}, {gy:.2f})")
        
        # Plan path using A*
        self.plan_path(self.robot_x, self.robot_y, gx, gy)

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        rospy.loginfo(f"Planning from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")
        
        start_r, start_c = self.world_to_grid(start_x, start_y)
        goal_r, goal_c = self.world_to_grid(goal_x, goal_y)
        
        # Bounds check
        if not (0 <= start_r < self.grid_size_cells and 0 <= start_c < self.grid_size_cells):
            rospy.logerr("Start pose is out of grid boundaries!")
            return
        if not (0 <= goal_r < self.grid_size_cells and 0 <= goal_c < self.grid_size_cells):
            rospy.logerr("Goal pose is out of grid boundaries!")
            return
            
        # Collision check
        if self.grid[start_r, start_c] == 1:
            rospy.logerr("Start pose is inside an obstacle cell!")
            return
        if self.grid[goal_r, goal_c] == 1:
            rospy.logerr("Goal pose is inside an obstacle cell!")
            return

        # A* Search
        start_time = time.time()
        
        open_set = []
        heapq.heappush(open_set, (0.0, (start_r, start_c)))
        
        parent = {}
        g_score = {(start_r, start_c): 0.0}
        closed_set = set()
        
        if self.allow_diagonal:
            directions = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
            ]
        else:
            directions = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0)
            ]

        def heuristic(r, c):
            # Manhattan distance (as requested)
            return abs(r - goal_r) + abs(c - goal_c)

        found = False
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == (goal_r, goal_c):
                found = True
                break
                
            if current in closed_set:
                continue
            closed_set.add(current)
            
            r, c = current
            curr_g = g_score[current]
            
            for dr, dc, step_cost in directions:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.grid_size_cells and 0 <= nc < self.grid_size_cells:
                    if self.grid[nr, nc] == 1:
                        continue
                        
                    potential_cost = self.potentials[nr, nc]
                    tentative_g = curr_g + step_cost + potential_cost
                    
                    neighbor = (nr, nc)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(nr, nc)
                        heapq.heappush(open_set, (f_score, neighbor))
                        parent[neighbor] = current

        exec_duration = time.time() - start_time
        
        if not found:
            rospy.logerr(f"A* failed to find path. Search time: {exec_duration * 1000:.2f} ms")
            return
            
        rospy.loginfo(f"A* succeeded! Time: {exec_duration * 1000:.2f} ms, Nodes: {len(closed_set)}")
        
        # Reconstruct path
        path_cells = []
        curr = (goal_r, goal_c)
        while curr in parent:
            path_cells.append(curr)
            curr = parent[curr]
        path_cells.append((start_r, start_c))
        path_cells.reverse()
        
        self.planned_path = [self.grid_to_world(r, c) for r, c in path_cells]
        
        # Publish path for RVIZ
        self.publish_path()
        
        # Initialize Path Follower
        self.path_follower = PathFollowerController(self.planned_path, self.kv, self.kw, self.lookahead, self.goal_tol)
        self.navigating = True

    def publish_path(self):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_msg.header.frame_id # usually 'map'
        
        for wx, wy in self.planned_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # default orient
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)

    def run(self):
        rate = rospy.Rate(20) # 20 Hz
        
        while not rospy.is_shutdown():
            if self.navigating and self.path_follower:
                # Get speed commands
                v, w, done = self.path_follower.get_control(self.robot_x, self.robot_y, self.robot_theta)
                
                # Publish twist
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = w
                self.cmd_vel_pub.publish(twist)
                
                if done:
                    rospy.loginfo("Goal Reached! Stopping robot.")
                    self.navigating = False
                    self.planned_path = []
                    # Publish zero velocity
                    self.cmd_vel_pub.publish(Twist())
            rate.sleep()

class PathFollowerController:
    def __init__(self, path, kv, kw, lookahead, goal_tol):
        self.path = path
        self.kv = kv
        self.kw = kw
        self.lookahead = lookahead
        self.goal_tol = goal_tol
        self.target_idx = 0

    def get_control(self, rx, ry, rtheta):
        if not self.path:
            return 0.0, 0.0, True
            
        goal_x, goal_y = self.path[-1]
        dist_to_goal = math.hypot(goal_x - rx, goal_y - ry)
        
        if dist_to_goal < self.goal_tol:
            return 0.0, 0.0, True
            
        # Find lookahead target point
        target_pt = self.path[-1]
        for idx in range(self.target_idx, len(self.path)):
            wx, wy = self.path[idx]
            dist = math.hypot(wx - rx, wy - ry)
            if dist >= self.lookahead:
                self.target_idx = idx
                target_pt = (wx, wy)
                break
                
        tx, ty = target_pt
        
        # Calculate angle and heading error
        target_angle = math.atan2(ty - ry, tx - rx)
        angle_err = math.atan2(math.sin(target_angle - rtheta), math.cos(target_angle - rtheta))
        
        # Control law
        if abs(angle_err) > 0.5:
            # Rotate in place if heading error is large
            v = 0.0
            w = self.kw * np.sign(angle_err) * 0.5
        else:
            # Move forward and rotate
            v = self.kv * min(dist_to_goal, 0.4)
            v = v * math.cos(angle_err)
            w = self.kw * angle_err
            
        # Limits
        v = np.clip(v, -0.05, 0.35)
        w = np.clip(w, -0.8, 0.8)
        
        return float(v), float(w), False

if __name__ == '__main__':
    try:
        node = RosAstarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
