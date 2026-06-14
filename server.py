import http.server
import json
import urllib.parse
import sys
import os
from path_planner import PathPlanner, PathFollower

# Add current path to import path if needed
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Define map file location
MAP_YAML = "/home/luiz/Área de trabalho/map.yaml"

class SimulationServer(http.server.BaseHTTPRequestHandler):
    planner = None
    
    @classmethod
    def get_planner(cls):
        if cls.planner is None:
            print("Initializing PathPlanner...")
            cls.planner = PathPlanner(MAP_YAML)
        return cls.planner

    def _send_cors_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def do_OPTIONS(self):
        self.send_response(200)
        self._send_cors_headers()
        self.end_headers()

    def do_GET(self):
        self._send_cors_headers()
        parsed_url = urllib.parse.urlparse(self.path)
        
        if parsed_url.path == "/api/map":
            try:
                planner = self.get_planner()
                # Prepare grid and potentials for JSON transmission
                # Downsample slightly to 150x150 for faster network transmission if needed,
                # but 300x300 is only 90k elements which is very small (~200KB JSON).
                grid_list = planner.grid.tolist()
                potentials_list = planner.potentials.tolist()
                
                response_data = {
                    "grid": grid_list,
                    "potentials": potentials_list,
                    "resolution": planner.resolution,
                    "origin_x": planner.grid_origin_x,
                    "origin_y": planner.grid_origin_y,
                    "size_cells": planner.grid_size_cells,
                    "original_origin": planner.origin,
                    "original_size": [planner.width, planner.height]
                }
                
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(response_data).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

    def do_POST(self):
        self._send_cors_headers()
        parsed_url = urllib.parse.urlparse(self.path)
        
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length)
        
        try:
            data = json.loads(body.decode("utf-8")) if body else {}
        except json.JSONDecodeError:
            self.send_response(400)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"error": "Invalid JSON"}).encode("utf-8"))
            return

        if parsed_url.path == "/api/plan":
            try:
                start = data.get("start") # [x, y]
                goal = data.get("goal")   # [x, y]
                allow_diagonal = data.get("allow_diagonal", False)
                inflation_radius = data.get("inflation_radius", 1.0)
                u_weight = data.get("u_weight", 20.0)
                
                if not start or not goal:
                    raise ValueError("Start and goal parameters are required.")
                
                planner = self.get_planner()
                # Recalculate potentials if parameters changed
                planner.potentials = planner.compute_potentials(
                    planner.dist_grid, 
                    inflation_radius=inflation_radius, 
                    u_weight=u_weight
                )
                
                path, meta = planner.astar(start, goal, allow_diagonal=allow_diagonal)
                
                if path is None:
                    response_data = {"success": False, "error": meta.get("error", "Unknown planning error")}
                else:
                    response_data = {
                        "success": True,
                        "path": path,
                        "metadata": {
                            "time_ms": meta["time"] * 1000,
                            "path_length_m": meta["path_length_m"],
                            "expanded_count": meta["expanded_count"],
                            "expanded_nodes": meta["expanded_nodes"],
                            "path_cells": meta["path_cells"]
                        }
                    }
                
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(response_data).encode("utf-8"))
                
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
                
        elif parsed_url.path == "/api/simulate":
            try:
                path = data.get("path") # list of [x, y]
                start_pose = data.get("start_pose") # [x, y, theta]
                kv = data.get("kv", 0.5)
                kw = data.get("kw", 1.2)
                lookahead = data.get("lookahead", 0.4)
                goal_tol = data.get("goal_tol", 0.15)
                
                if not path or not start_pose:
                    raise ValueError("Path and start_pose parameters are required.")
                
                follower = PathFollower(path, kv=kv, kw=kw, lookahead=lookahead, goal_tol=goal_tol)
                
                rx, ry, rtheta = start_pose
                trajectory = []
                dt = 0.05 # 50ms time step
                max_steps = 1000
                
                # Append initial pose
                trajectory.append({
                    "x": rx, "y": ry, "theta": rtheta,
                    "v": 0.0, "w": 0.0, "tx": rx, "ty": ry, "err": 0.0
                })
                
                step = 0
                while step < max_steps:
                    v, w, target_pt, done = follower.get_control(rx, ry, rtheta)
                    if done:
                        break
                        
                    # Update robot kinematics
                    rx = rx + v * math_cos(rtheta) * dt
                    ry = ry + v * math_sin(rtheta) * dt
                    rtheta = rtheta + w * dt
                    # Normalize theta between -pi and pi
                    rtheta = (rtheta + 3.14159265) % (2 * 3.14159265) - 3.14159265
                    
                    err = float(np.hypot(target_pt[0] - rx, target_pt[1] - ry))
                    
                    trajectory.append({
                        "x": float(rx),
                        "y": float(ry),
                        "theta": float(rtheta),
                        "v": float(v),
                        "w": float(w),
                        "tx": float(target_pt[0]),
                        "ty": float(target_pt[1]),
                        "err": err
                    })
                    step += 1
                
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"success": True, "trajectory": trajectory}).encode("utf-8"))
            except Exception as e:
                self.send_response(500)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(e)}).encode("utf-8"))
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"Not Found")

import math
def math_cos(x): return math.cos(x)
def math_sin(x): return math.sin(x)

def run_server(port=5000):
    server_address = ('', port)
    httpd = http.server.HTTPServer(server_address, SimulationServer)
    print(f"Path Planning Simulator backend running on port {port}...")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping backend server.")
        httpd.server_close()

if __name__ == "__main__":
    port = 5000
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            pass
    run_server(port)
