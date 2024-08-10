import argparse
import re

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.animation import FuncAnimation

# Argument parser
parser = argparse.ArgumentParser()
parser.add_argument("--mapname", "-m", type=str, required=True, help="Name of the map")
parser.add_argument("--obs", "-o", type=str, required=True, help="Obstacle configuration")
parser.add_argument("--robotnum", "-r", type=str, required=True, help="Number of robots")
parser.add_argument("--testnum", "-t", type=str, required=True, help="Test number")
parser.add_argument("--interval", "-i", type=float, default=0.1, help="Time interval for updates")
args = parser.parse_args()

mapname = args.mapname
obs = args.obs
robotnum = args.robotnum
testnum = args.testnum
interval = 0.1

benchmarkPath = f"benchmark/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}.yaml"
solutionPath = f"over_solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_solution.txt"

# Load the YAML file
with open(benchmarkPath, 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

# Load the solution file
with open(solutionPath, 'r') as f:
    solution = f.read()

# Parse paths for each agent
paths = []
for path_str in solution.split('Agent')[1:]:
    path = []
    for point_str in re.findall(r'\(.*?\)', path_str):
        x, y, t = map(float, point_str.strip('()').split(','))
        path.append((x, y, t))
    paths.append(path)

# Parse obstacles
obstacles = data.get('obstacles', [])

# Calculate number of animation frames
max_time = max(point[2] for path in paths for point in path)
num_frames = int(max_time / interval) + 1

# Initialize plot
fig, ax = plt.subplots(figsize=(8, 8))
radius = 0.5  # Fixed radius for agents
agents = [patches.Circle((0, 0), radius, color='blue', fill=True) for _ in range(len(paths))]
agent_labels = [ax.text(0, 0, '', fontsize=8, color='white', ha='center', va='center') for _ in range(len(paths))]
time_text = ax.text(0.005, 0.995, '', transform=ax.transAxes, horizontalalignment='left', verticalalignment='top')

# Add agents to the plot
for agent in agents:
    ax.add_patch(agent)

# Mark start and goal points with agent ID
for i, path in enumerate(paths):
    start_x, start_y, _ = path[0]
    goal_x, goal_y, _ = path[-1]
    ax.plot(start_x, start_y, marker='s', markersize=10, color='green')
    ax.plot(goal_x, goal_y, marker='*', markersize=10, color='red')
    ax.text(start_x, start_y, f'S{i}', fontsize=8, color='black', ha='right', va='bottom')
    ax.text(goal_x, goal_y, f'G{i}', fontsize=8, color='black', ha='right', va='bottom')

# Add obstacles to the plot
for obs in obstacles:
    if 'radius' in obs:
        circle = patches.Circle(obs['center'], obs['radius'], color='gray', fill=True)
        ax.add_patch(circle)
    elif 'width' in obs and 'height' in obs:
        rect = patches.Rectangle((obs['center'][0] - obs['width'] / 2, obs['center'][1] - obs['height'] / 2),
            obs['width'], obs['height'], color='gray', fill=True)
        ax.add_patch(rect)


def init():
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 40)
    ax.set_aspect('equal')
    for agent in agents:
        agent.center = (0, 0)
        agent.set_color('blue')  # Reset color to blue in init
    for label in agent_labels:
        label.set_text('')
    time_text.set_text('')
    return agents + agent_labels + [time_text]


def update_agents_positions(current_time):
    for i, path in enumerate(paths):
        for j in range(len(path) - 1):
            if path[j][2] <= current_time < path[j + 1][2]:
                start = np.array(path[j][:2])
                end = np.array(path[j + 1][:2])
                ratio = (current_time - path[j][2]) / (path[j + 1][2] - path[j][2])
                current_x = start[0] + ratio * (end[0] - start[0])
                current_y = start[1] + ratio * (end[1] - start[1])
                agents[i].center = (current_x, current_y)
                agent_labels[i].set_text(str(i))
                agent_labels[i].set_position((current_x, current_y))
                break
        else:
            current_x, current_y = path[-1][:2]
            agents[i].center = (current_x, current_y)
            agent_labels[i].set_text(str(i))
            agent_labels[i].set_position((current_x, current_y))


def detect_collisions(current_time):
    for i, agent1 in enumerate(agents):
        for j, agent2 in enumerate(agents):
            if i != j:
                distance = np.linalg.norm(np.array(agent1.center) - np.array(agent2.center))
                if distance < 2 * (radius * 0.9):
                    agents[i].set_color('red')
                    agents[j].set_color('red')
                    print(f"Collision detected between agent {i} and agent {j} at time {current_time:.2f}")


def update(frame):
    current_time = frame * interval
    time_text.set_text(f'Time: {current_time:.2f}')

    update_agents_positions(current_time)
    detect_collisions(current_time)

    return agents + agent_labels + [time_text]


# Calculate the speed multiplier to keep the animation speed consistent
animation_interval = interval * 10

ani = FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=animation_interval)

plt.show()
