import argparse
import re

import matplotlib
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml
import matplotlib.animation as animation
from tqdm import tqdm


matplotlib.use('TkAgg')

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
solutionPath = f"solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_solution.txt"
controlPath = f"control/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_controls.txt"

# Load the YAML file
with open(benchmarkPath, 'r') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

# Load the solution file
with open(solutionPath, 'r') as f:
    solution = f.read()

# Load the control file
with open(controlPath, 'r') as f:
    control = f.read()

# Parse paths for each agent
paths = []
for path_str in solution.split('Agent')[1:]:
    path = []
    for point_str in re.findall(r'\(.*?\)', path_str):
        x, y, t = map(float, point_str.strip('()').split(','))
        path.append((x, y, t))
    paths.append(path)

# Parse control for each agent
controls = []
for control_str in control.split('Agent')[1:]:
    agent_controls = []
    for control_point in re.findall(r'\(.*?\)', control_str):
        acc_x, acc_y, t1_x, t1_y, t2_x, t2_y = map(float, control_point.strip('()').split(','))
        agent_controls.append((acc_x, acc_y, t1_x, t1_y, t2_x, t2_y))
    controls.append(agent_controls)

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


def calculate_state(initial_state, acc_time, acceleration, elapsed_time):
    x0, y0 = initial_state
    t1x, t1y = acc_time
    ax, ay = acceleration

    def calc_coord(t, p0, t1, acc):
        if t <= t1:
            return p0 + 0.5 * acc * t * t
        else:
            x1 = p0 + 0.5 * acc * t1 * t1
            v1 = acc * t1
            dt = t - t1
            return x1 + v1 * dt - 0.5 * acc * dt * dt

    x = calc_coord(elapsed_time, x0, t1x, ax)
    y = calc_coord(elapsed_time, y0, t1y, ay)

    return (x, y)

def update_agents_positions(current_time):
    for i, (path, agent_controls) in enumerate(zip(paths, controls)):
        for j in range(len(path) - 1):
            if path[j][2] <= current_time < path[j + 1][2]:
                start = path[j][:2]
                control = agent_controls[j + 1]
                acc_x, acc_y, t1_x, t1_y, t2_x, t2_y = control

                if all(v == 0 for v in control):
                    # If all control values are zero, keep the agent at its current position
                    current_x, current_y = start
                else:
                    elapsed_time = current_time - path[j][2]
                    current_x, current_y = calculate_state(start, (t1_x, t1_y), (acc_x, acc_y), elapsed_time)

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

ani = animation.FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=animation_interval)

# Set up the writer
Writer = animation.writers['ffmpeg']
writer = Writer(fps=30, metadata=dict(artist='Me'), bitrate=1800)

progress_bar = tqdm(total=num_frames, unit='frames')

def progress_callback(current_frame, total_frames):
    progress_bar.n = current_frame
    progress_bar.refresh()

# Save the animation
output_file = f"mapf_dynamics.mp4"
ani.save(output_file, writer=writer, progress_callback=progress_callback)

progress_bar.close()

print(f"Animation saved as {output_file}")

plt.show()
