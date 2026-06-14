import os
import time
import matplotlib.pyplot as plt
import numpy as np
from path_planner import PathPlanner

MAP_YAML = "/home/luiz/Área de trabalho/map.yaml"

def main():
    print("Initializing PathPlanner...")
    planner = PathPlanner(MAP_YAML)
    
    # Coordinates in world frame
    # Start: (4.1, 0.6), Goal: (14.1, 8.9)
    start_pose = (4.1, 0.6)
    goal_pose = (14.1, 8.9)
    
    print(f"Planning path from {start_pose} to {goal_pose}...")
    
    # Test 1: 4-Way Connected (strictly Manhattan)
    t0 = time.time()
    path_4way, meta_4way = planner.astar(start_pose, goal_pose, allow_diagonal=False)
    t1 = time.time()
    
    if path_4way:
        print(f"4-Way path found!")
        print(f"  Execution time: {(t1 - t0)*1000:.2f} ms")
        print(f"  Path length: {meta_4way['path_length_m']:.2f} meters")
        print(f"  Nodes expanded: {meta_4way['expanded_count']}")
    else:
        print(f"4-Way path planning failed! Error: {meta_4way.get('error', 'Unknown error')}")
        
    # Test 2: 8-Way Connected (Diagonal allowed)
    t0 = time.time()
    path_8way, meta_8way = planner.astar(start_pose, goal_pose, allow_diagonal=True)
    t1 = time.time()
    
    if path_8way:
        print(f"8-Way path found!")
        print(f"  Execution time: {(t1 - t0)*1000:.2f} ms")
        print(f"  Path length: {meta_8way['path_length_m']:.2f} meters")
        print(f"  Nodes expanded: {meta_8way['expanded_count']}")
    else:
        print(f"8-Way path planning failed! Error: {meta_8way.get('error', 'Unknown error')}")

    # Generate visualization plot
    print("Generating path plot...")
    plt.figure(figsize=(10, 10))
    
    # Show grid: 0 is free, 1 is occupied
    grid_display = np.array(planner.grid, dtype=float)
    # Highlight potential fields
    # Let's map potentials to a heatmap
    display_img = np.zeros((planner.grid_size_cells, planner.grid_size_cells, 3))
    
    for r in range(planner.grid_size_cells):
        for c in range(planner.grid_size_cells):
            if planner.grid[r, c] == 1:
                display_img[r, c] = [0.1, 0.1, 0.15] # Dark slate for obstacles
            else:
                pot = planner.potentials[r, c]
                if pot > 0.05:
                    intensity = min(pot / 20.0, 1.0)
                    display_img[r, c] = [0.8 * intensity, 0.2 * intensity, 0.2] # Reddish for potential field
                else:
                    display_img[r, c] = [0.9, 0.9, 0.9] # Light gray for free space
                    
    # Draw grid. Origin is bottom-left, so we set origin='lower' to match ROS coordinate frame
    plt.imshow(display_img, origin='lower', extent=[
        planner.grid_origin_x, planner.grid_origin_x + planner.grid_size_m,
        planner.grid_origin_y, planner.grid_origin_y + planner.grid_size_m
    ])
    
    # Plot expanded nodes for 4-way
    if 'expanded_nodes' in meta_4way:
        exp_x = []
        exp_y = []
        for r, c in meta_4way['expanded_nodes']:
            wx, wy = planner.grid_to_world(r, c)
            exp_x.append(wx)
            exp_y.append(wy)
        plt.scatter(exp_x, exp_y, color='lightblue', s=1, alpha=0.3, label='Expanded Nodes (Frontier)')

    # Plot 4-way path
    if path_4way:
        px_4 = [x for x, y in path_4way]
        py_4 = [y for x, y in path_4way]
        plt.plot(px_4, py_4, color='green', linewidth=2, label='Planned Path (A* 4-Way)')

    # Plot start and goal
    plt.scatter([start_pose[0]], [start_pose[1]], color='green', s=100, zorder=5, label='Start Position')
    plt.scatter([goal_pose[0]], [goal_pose[1]], color='red', s=100, marker='X', zorder=5, label='Goal Position')
    
    plt.title(f"A* Path Planning & Potential Fields (30m x 30m Grid)\n"
              f"Path Length: {meta_4way['path_length_m']:.2f}m | Planning Time: {meta_4way['time']*1000:.2f}ms")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    plt.legend()
    
    output_png = "/home/luiz/.gemini/antigravity/scratch/path_test.png"
    plt.savefig(output_png, dpi=150)
    print(f"Verification plot saved to: {output_png}")

if __name__ == "__main__":
    main()
