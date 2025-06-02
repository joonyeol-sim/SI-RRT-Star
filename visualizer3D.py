import argparse
import re

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import yaml
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser()
parser.add_argument("--mapname", "-m", type=str, required=True, help="Name of the map")
parser.add_argument(
    "--obs", "-o", type=str, required=True, help="Obstacle configuration"
)
parser.add_argument(
    "--robotnum", "-r", type=str, required=True, help="Number of robots"
)
parser.add_argument("--testnum", "-t", type=str, required=True, help="Test number")
parser.add_argument(
    "--interval", "-i", type=float, default=0.1, help="Time interval for updates"
)
args = parser.parse_args()

mapname = args.mapname
obs = args.obs
robotnum = args.robotnum
testnum = args.testnum
interval = args.interval

benchmarkPath = f"benchmark/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}.yaml"
solutionPath = f"solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_solution.txt"

with open(benchmarkPath, "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

with open(solutionPath, "r") as f:
    solution = f.read()

paths = []
for path_str in solution.split("Agent")[1:]:
    path = []
    for point_str in re.findall(r"\(.*?\)", path_str):
        coords = point_str.strip("()").split(",")
        if len(coords) == 4:  # 3D coordinates (x, y, z, t)
            x, y, z, t = map(float, coords)
            path.append((x, y, z, t))
        elif len(coords) == 3:  # 2D coordinates (x, y, t) - add z=0
            x, y, t = map(float, coords)
            path.append((x, y, 0.0, t))
    paths.append(path)

obstacles = data.get("obstacles", [])

start_points = data.get("startPoints", [])
goal_points = data.get("goalPoints", [])

# Handle both 2D and 3D start/goal points
for i, start in enumerate(start_points):
    if len(start) == 2:
        start_points[i] = [start[0], start[1], 0.0]  # Add z=0 for 2D points

for i, goal in enumerate(goal_points):
    if len(goal) == 2:
        goal_points[i] = [goal[0], goal[1], 0.0]  # Add z=0 for 2D points

max_time = max(point[3] for path in paths for point in path)
num_frames = int(max_time / interval) + 1

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
radius = 0.5

# Store agent positions for animation
agent_positions = [(0, 0, 0) for _ in range(len(paths))]
agent_colors = ['blue'] * len(paths)

# Create scatter plot for agents (will be updated each frame)
agent_scatter = ax.scatter([0]*len(paths), [0]*len(paths), [0]*len(paths), s=200, c=agent_colors, alpha=0.8)

# Text for time display
time_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=12)

# Plot start and goal points
for i, (start, goal) in enumerate(zip(start_points, goal_points)):
    start_x, start_y, start_z = start
    goal_x, goal_y, goal_z = goal
    
    # Plot goal points as stars
    ax.scatter(goal_x, goal_y, goal_z, marker='*', s=200, c='red', label=f'Goal {i}' if i == 0 else "")
    ax.text(goal_x, goal_y, goal_z, f'G{i}', fontsize=8)

# Plot obstacles
for obs in obstacles:
    if "radius" in obs:  # Spherical obstacle
        center = obs["center"]
        if len(center) == 2:
            center = [center[0], center[1], 0.0]  # Add z=0 for 2D obstacles
        
        # Create sphere
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x_sphere = obs["radius"] * np.outer(np.cos(u), np.sin(v)) + center[0]
        y_sphere = obs["radius"] * np.outer(np.sin(u), np.sin(v)) + center[1]
        z_sphere = obs["radius"] * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
        ax.plot_surface(x_sphere, y_sphere, z_sphere, alpha=0.3, color='gray')
        
    elif "width" in obs and "height" in obs:  # Rectangular obstacle
        center = obs["center"]
        if len(center) == 2:
            center = [center[0], center[1], 0.0]  # Add z=0 for 2D obstacles
        
        width = obs["width"]
        height = obs["height"]
        depth = obs.get("depth", 1.0)  # Default depth for 3D box
        
        # Create box vertices
        x_min, x_max = center[0] - width/2, center[0] + width/2
        y_min, y_max = center[1] - height/2, center[1] + height/2
        z_min, z_max = center[2] - depth/2, center[2] + depth/2
        
        # Draw box edges
        vertices = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_max, y_max, z_max], [x_min, y_max, z_max]
        ]
        
        # Define the 12 edges of a cube
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # top face
            [0, 4], [1, 5], [2, 6], [3, 7]   # vertical edges
        ]
        
        for edge in edges:
            points = np.array([vertices[edge[0]], vertices[edge[1]]])
            ax.plot3D(points[:, 0], points[:, 1], points[:, 2], 'gray', alpha=0.6, linewidth=2)

def init():
    # Set axis limits - adjust based on your environment size
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 40)
    ax.set_zlim(0, 20)  # Adjust z-limit as needed
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('SI-RRT* 3D Visualization')
    
    # Initialize agent positions
    for i in range(len(paths)):
        agent_positions[i] = (0, 0, 0)
        agent_colors[i] = 'blue'
    
    time_text.set_text("")
    return [agent_scatter, time_text]

def update_agents_positions(current_time):
    global agent_positions, agent_colors
    
    # Reset colors
    agent_colors = ['blue'] * len(paths)
    
    for i, path in enumerate(paths):
        for j in range(len(path) - 1):
            if path[j][3] <= current_time < path[j + 1][3]:
                start = np.array(path[j][:3])
                end = np.array(path[j + 1][:3])
                ratio = (current_time - path[j][3]) / (path[j + 1][3] - path[j][3])
                current_pos = start + ratio * (end - start)
                agent_positions[i] = tuple(current_pos)
                break
        else:
            # Use final position
            agent_positions[i] = path[-1][:3]

def detect_collisions(current_time):
    global agent_colors
    
    for i in range(len(agent_positions)):
        for j in range(i + 1, len(agent_positions)):
            distance = np.linalg.norm(
                np.array(agent_positions[i]) - np.array(agent_positions[j])
            )
            if distance < 2 * radius:
                agent_colors[i] = 'red'
                agent_colors[j] = 'red'
                print(f"Collision detected between agent {i} and agent {j} at time {current_time:.2f}")

def update(frame):
    current_time = frame * interval
    time_text.set_text(f"Time: {current_time:.2f}")
    
    update_agents_positions(current_time)
    detect_collisions(current_time)
    
    # Update scatter plot
    if agent_positions:
        positions = np.array(agent_positions)
        agent_scatter._offsets3d = (positions[:, 0], positions[:, 1], positions[:, 2])
        agent_scatter.set_color(agent_colors)
    
    return [agent_scatter, time_text]

animation_interval = interval * 100  # Adjust for smoother animation

ani = animation.FuncAnimation(
    fig,
    update,
    frames=num_frames,
    init_func=init,
    blit=False,  # Set to False for 3D animations
    interval=animation_interval,
)

plt.show()