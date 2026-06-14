import os
import yaml
import numpy as np
from PIL import Image
import heapq
import collections
import time

class PathPlanner:
    def __init__(self, map_yaml_path):
        self.map_yaml_path = map_yaml_path
        self.map_dir = os.path.dirname(map_yaml_path)
        
        # Load map metadata
        with open(map_yaml_path, 'r') as f:
            self.metadata = yaml.safe_load(f)
            
        self.resolution = float(self.metadata['resolution'])
        self.origin = [float(v) for v in self.metadata['origin']]
        
        # Load image file
        image_relative_path = self.metadata['image']
        # The path in map.yaml is /home/anderson/map.pgm, but it is actually on the Desktop
        # Let's handle absolute path vs local directory replacement.
        image_filename = os.path.basename(image_relative_path)
        self.image_path = os.path.join(self.map_dir, image_filename)
        
        if not os.path.exists(self.image_path):
            raise FileNotFoundError(f"Map image not found at: {self.image_path}")
            
        self.img = Image.open(self.image_path)
        self.map_data = np.array(self.img)
        self.height, self.width = self.map_data.shape
        
        # Define 30m x 30m grid window
        # Center it at X=10.7, Y=4.2 to cover the free space range [X: -0.1 to 21.5, Y: -3.3 to 11.7]
        self.grid_size_m = 30.0
        self.grid_size_cells = int(self.grid_size_m / self.resolution)  # 300 cells
        
        self.grid_x_center = 10.7
        self.grid_y_center = 4.2
        
        # Calculate origin of the 30m x 30m grid in world coordinates
        self.grid_origin_x = self.grid_x_center - (self.grid_size_m / 2.0) # -4.3
        self.grid_origin_y = self.grid_y_center - (self.grid_size_m / 2.0) # -10.8
        
        # Find map coordinate offsets
        self.col_min = int((self.grid_origin_x - self.origin[0]) / self.resolution)
        self.row_min = int((self.grid_origin_y - self.origin[1]) / self.resolution)
        
        # Construct the 30x30m grid (binary: 1=occupied, 0=free)
        self.grid = np.ones((self.grid_size_cells, self.grid_size_cells), dtype=np.uint8)
        
        for r in range(self.grid_size_cells):
            map_row = self.row_min + r
            if map_row < 0 or map_row >= self.height:
                continue
            # Image y is top-down, ROS map y is bottom-up
            img_row = self.height - 1 - map_row
            
            for c in range(self.grid_size_cells):
                map_col = self.col_min + c
                if map_col < 0 or map_col >= self.width:
                    continue
                
                pixel_val = self.map_data[img_row, map_col]
                # In Gmapping: 0 is occupied (black), 205 is unknown, 254 is free.
                # We treat occupied and unknown as obstacles for safe navigation.
                if pixel_val == 254:
                    self.grid[r, c] = 0 # Free
                else:
                    self.grid[r, c] = 1 # Occupied/Unknown

        # Compute the potential field (distance transform & cost inflation)
        self.dist_grid = self.compute_distance_transform()
        self.potentials = self.compute_potentials(self.dist_grid)

    def world_to_grid(self, x, y):
        """Converts world coordinate (x, y) to grid indices (r, c)"""
        c = int((x - self.grid_origin_x) / self.resolution)
        r = int((y - self.grid_origin_y) / self.resolution)
        return r, c

    def grid_to_world(self, r, c):
        """Converts grid indices (r, c) to world coordinate (x, y) at cell center"""
        x = self.grid_origin_x + (c + 0.5) * self.resolution
        y = self.grid_origin_y + (r + 0.5) * self.resolution
        return x, y

    def compute_distance_transform(self):
        """Computes distance to nearest obstacle using multi-source BFS"""
        dist = np.full(self.grid.shape, 1e9, dtype=np.float32)
        q = collections.deque()
        
        # Enqueue all obstacles
        for r in range(self.grid_size_cells):
            for c in range(self.grid_size_cells):
                if self.grid[r, c] == 1:
                    dist[r, c] = 0.0
                    q.append((r, c))
                    
        # Multi-source BFS
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
                        
        return dist * self.resolution

    def compute_potentials(self, dist_grid, inflation_radius=1.0, u_weight=20.0):
        """Computes potential field (repulsive force near obstacles)"""
        potentials = np.zeros(dist_grid.shape, dtype=np.float32)
        
        # Cells within inflation_radius get potential cost
        mask = dist_grid < inflation_radius
        # Formula: U = weight * (1 - d/R)^2
        # If d == 0 (obstacle itself), we don't plan through it anyway because A* checks collison.
        potentials[mask] = u_weight * (1.0 - dist_grid[mask] / inflation_radius) ** 2
        return potentials

    def astar(self, start_world, goal_world, allow_diagonal=False):
        """Runs A* search with Manhattan distance heuristic on the 30x30m grid"""
        start_time = time.time()
        
        start_r, start_c = self.world_to_grid(start_world[0], start_world[1])
        goal_r, goal_c = self.world_to_grid(goal_world[0], goal_world[1])
        
        # Verify starts and goals are within bounds
        if not (0 <= start_r < self.grid_size_cells and 0 <= start_c < self.grid_size_cells):
            return None, {"error": "Start position is out of bounds.", "time": 0}
        if not (0 <= goal_r < self.grid_size_cells and 0 <= goal_c < self.grid_size_cells):
            return None, {"error": "Goal position is out of bounds.", "time": 0}
            
        # Verify they are not in obstacles
        if self.grid[start_r, start_c] == 1:
            return None, {"error": "Start position is inside an obstacle.", "time": 0}
        if self.grid[goal_r, goal_c] == 1:
            return None, {"error": "Goal position is inside an obstacle.", "time": 0}

        # Priority Queue element format: (f_score, (r, c))
        open_set = []
        heapq.heappush(open_set, (0.0, (start_r, start_c)))
        
        parent = {}
        g_score = { (start_r, start_c): 0.0 }
        
        closed_set = set()
        expanded_nodes = [] # for visualization

        # Directions: 4-connectivity vs 8-connectivity
        if allow_diagonal:
            directions = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0),
                (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
            ]
        else:
            directions = [
                (0, 1, 1.0), (0, -1, 1.0), (1, 0, 1.0), (-1, 0, 1.0)
            ]

        def heuristic(r, c):
            # Manhattan distance (as requested by the assignment)
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
            expanded_nodes.append(current)
            
            r, c = current
            curr_g = g_score[current]
            
            for dr, dc, step_cost in directions:
                nr, nc = r + dr, c + dc
                
                if 0 <= nr < self.grid_size_cells and 0 <= nc < self.grid_size_cells:
                    # Check collision
                    if self.grid[nr, nc] == 1:
                        continue
                        
                    # Total step cost = movement cost + potential field cost
                    potential_cost = self.potentials[nr, nc]
                    tentative_g = curr_g + step_cost + potential_cost
                    
                    neighbor = (nr, nc)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        g_score[neighbor] = tentative_g
                        f_score = tentative_g + heuristic(nr, nc)
                        heapq.heappush(open_set, (f_score, neighbor))
                        parent[neighbor] = current

        exec_time = time.time() - start_time
        
        if not found:
            return None, {
                "error": "Path not found.",
                "time": exec_time,
                "expanded_count": len(closed_set)
            }
            
        # Reconstruct path
        path_cells = []
        curr = (goal_r, goal_c)
        while curr in parent:
            path_cells.append(curr)
            curr = parent[curr]
        path_cells.append((start_r, start_c))
        path_cells.reverse()
        
        # Convert path to world coordinates
        path_world = [self.grid_to_world(r, c) for r, c in path_cells]
        
        metadata = {
            "time": exec_time,
            "path_length_m": len(path_world) * self.resolution,
            "expanded_count": len(closed_set),
            "expanded_nodes": [[int(r), int(c)] for r, c in expanded_nodes],
            "path_cells": [[int(r), int(c)] for r, c in path_cells]
        }
        
        return path_world, metadata

class PathFollower:
    def __init__(self, path_world, kv=0.5, kw=1.2, lookahead=0.4, goal_tol=0.15):
        self.path = path_world
        self.kv = kv
        self.kw = kw
        self.lookahead = lookahead
        self.goal_tol = goal_tol
        self.target_idx = 0

    def get_control(self, rx, ry, rtheta):
        """
        Computes control commands (v, w) using a Pure Pursuit / Heading controller.
        Returns:
            v: linear velocity (m/s)
            w: angular velocity (rad/s)
            target_pt: (x, y) target point
            done: boolean (True if goal reached)
        """
        if not self.path or len(self.path) == 0:
            return 0.0, 0.0, (rx, ry), True
            
        goal_x, goal_y = self.path[-1]
        dist_to_goal = np.hypot(goal_x - rx, goal_y - ry)
        
        if dist_to_goal < self.goal_tol:
            return 0.0, 0.0, (goal_x, goal_y), True
            
        # Find target waypoint
        # Search for first point on path that is further than lookahead distance
        target_pt = self.path[-1]
        for idx in range(self.target_idx, len(self.path)):
            wx, wy = self.path[idx]
            dist = np.hypot(wx - rx, wy - ry)
            if dist >= self.lookahead:
                self.target_idx = idx
                target_pt = (wx, wy)
                break
                
        tx, ty = target_pt
        
        # Angle to target
        target_angle = np.arctan2(ty - ry, tx - rx)
        
        # Heading error
        angle_err = np.arctan2(np.sin(target_angle - rtheta), np.cos(target_angle - rtheta))
        
        # Control signals
        # If heading error is large, rotate in place first
        if abs(angle_err) > 0.5: # ~30 degrees
            v = 0.0
            w = self.kw * np.sign(angle_err) * 0.6
        else:
            # Scale linear velocity based on distance to goal, capped at max v
            v = self.kv * min(dist_to_goal, 0.5)
            # Decelerate if turning sharply
            v = v * np.cos(angle_err)
            w = self.kw * angle_err
            
        # Limit velocities
        v = np.clip(v, -0.1, 0.4) # limit to 0.4 m/s max forward speed
        w = np.clip(w, -1.0, 1.0) # limit to 1.0 rad/s max turn rate
        
        return float(v), float(w), target_pt, False
