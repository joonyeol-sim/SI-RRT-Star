
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
        parts = point_str.strip('()').split(',')
        if len(parts) == 4:
            x, y, angle, t = map(float, parts)
            path.append((x, y, angle, t))
        else:
            # Fallback for different format
            x, y, z, angle = map(float, parts)
            # Assume z is time in this case
            path.append((x, y, angle, z))
    paths.append(path)

# Parse control for each agent
controls = []
for control_str in control.split('Agent')[1:]:
    agent_controls = []
    for control_point in re.findall(r'\(.*?\)', control_str):
        acc_x, acc_y, t1_x, t1_y, t2_x, t2_y = map(float, control_point.strip('()').split(','))
        agent_controls.append((acc_x, acc_y, t1_x, t1_y, t2_x, t2_y))
    controls.append(agent_controls)

# Extract start and goal states from benchmark
def extract_start_goal_from_benchmark(data):
    """Extract start and goal states from benchmark data"""
    start_states = []
    goal_states = []
    start_angles = []
    goal_angles = []

    agents_data = data['agents']
    for agent in agents_data:
        start_pos = agent['startState']['position']
        goal_pos = agent['goalState']['position']
        start_angle = agent['startState'].get('angle', 0)
        goal_angle = agent['goalState'].get('angle', 0)
        start_states.append(start_pos)
        goal_states.append(goal_pos)
        start_angles.append(start_angle)
        goal_angles.append(goal_angle)

    return start_states, goal_states, start_angles, goal_angles

# Extract start and goal states from benchmark
benchmark_start_states, benchmark_goal_states, benchmark_start_angles, benchmark_goal_angles = extract_start_goal_from_benchmark(data)

# Validation: Check if solution start/goal states match benchmark
def validate_solution_against_benchmark():
    """Validate that solution start/goal states match benchmark"""
    tolerance = 1e-2
    validation_passed = True

    print("🔍 Validating solution against benchmark...")

    if len(paths) != len(benchmark_start_states):
        print(f"❌ Number of agents mismatch: solution has {len(paths)}, benchmark has {len(benchmark_start_states)}")
        validation_passed = False

    for i, path in enumerate(paths):
        if i >= len(benchmark_start_states):
            break

        # Check start state
        solution_start = path[0][:2]  # (x, y) from solution
        benchmark_start = benchmark_start_states[i]
        start_diff = np.linalg.norm(np.array(solution_start) - np.array(benchmark_start))

        if start_diff > tolerance:
            print(f"❌ Agent {i} start state mismatch:")
            print(f"   Solution: ({solution_start[0]:.6f}, {solution_start[1]:.6f})")
            print(f"   Benchmark: ({benchmark_start[0]:.6f}, {benchmark_start[1]:.6f})")
            print(f"   Difference: {start_diff:.6f}")
            validation_passed = False

        # Check goal state
        solution_goal = path[-1][:2]  # (x, y) from solution
        benchmark_goal = benchmark_goal_states[i]
        goal_diff = np.linalg.norm(np.array(solution_goal) - np.array(benchmark_goal))

        if goal_diff > tolerance:
            print(f"❌ Agent {i} goal state mismatch:")
            print(f"   Solution: ({solution_goal[0]:.6f}, {solution_goal[1]:.6f})")
            print(f"   Benchmark: ({benchmark_goal[0]:.6f}, {benchmark_goal[1]:.6f})")
            print(f"   Difference: {goal_diff:.6f}")
            validation_passed = False

    if validation_passed:
        print("✅ Solution validation passed: All start/goal states match benchmark")
    else:
        print("❌ Solution validation failed: Some start/goal states don't match benchmark")

    return validation_passed

# Perform validation
validate_solution_against_benchmark()

# Parse obstacles
obstacles = data.get('obstacles', [])

# Get environment dimensions from benchmark
env_width = data.get('width', 40)
env_height = data.get('height', 40)
robot_radius = data.get('robotRadius', 0.5)

# Calculate number of animation frames - 시간 인덱스 수정
max_time = max(point[3] for path in paths for point in path)  # 시간은 인덱스 3
num_frames = int(max_time / interval) + 1

# Initialize plot
fig, ax = plt.subplots(figsize=(8, 8))
radius = robot_radius  # Use radius from benchmark

# Create agents (circles) and orientation lines
agents = [patches.Circle((0, 0), radius, color='blue', fill=True, alpha=0.7) for _ in range(len(paths))]
agent_lines = [ax.plot([], [], 'r-', linewidth=2)[0] for _ in range(len(paths))]  # orientation lines
agent_labels = [ax.text(0, 0, '', fontsize=8, color='white', ha='center', va='center') for _ in range(len(paths))]
time_text = ax.text(0.005, 0.995, '', transform=ax.transAxes, horizontalalignment='left', verticalalignment='top')

# Add agents to the plot
for agent in agents:
    ax.add_patch(agent)

# Mark start and goal points with agent ID and angles (using benchmark data)
for i in range(len(benchmark_start_states)):
    start_x, start_y = benchmark_start_states[i]
    goal_x, goal_y = benchmark_goal_states[i]
    start_angle = benchmark_start_angles[i]
    goal_angle = benchmark_goal_angles[i]

    ax.plot(start_x, start_y, marker='s', markersize=10, color='green')
    ax.plot(goal_x, goal_y, marker='*', markersize=10, color='red')
    ax.text(start_x, start_y, f'S{i}', fontsize=8, color='black', ha='right', va='bottom')
    ax.text(goal_x, goal_y, f'G{i}', fontsize=8, color='black', ha='right', va='bottom')

    # Draw start and goal orientation lines
    line_length = radius * 1.5
    start_line_x = start_x + line_length * np.cos(start_angle)
    start_line_y = start_y + line_length * np.sin(start_angle)
    goal_line_x = goal_x + line_length * np.cos(goal_angle)
    goal_line_y = goal_y + line_length * np.sin(goal_angle)

    ax.plot([start_x, start_line_x], [start_y, start_line_y], 'g-', linewidth=1, alpha=0.7)
    ax.plot([goal_x, goal_line_x], [goal_y, goal_line_y], 'r-', linewidth=1, alpha=0.7)

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
    ax.set_xlim(0, env_width)
    ax.set_ylim(0, env_height)
    ax.set_aspect('equal')

    # Initialize agents at their benchmark start positions
    for i, agent in enumerate(agents):
        if i < len(benchmark_start_states):
            start_x, start_y = benchmark_start_states[i]
            start_angle = benchmark_start_angles[i]
            agent.center = (start_x, start_y)

            # Initialize orientation line
            line_length = radius * 1.5
            line_x = start_x + line_length * np.cos(start_angle)
            line_y = start_y + line_length * np.sin(start_angle)
            agent_lines[i].set_data([start_x, line_x], [start_y, line_y])
        else:
            agent.center = (0, 0)
            agent_lines[i].set_data([], [])
        agent.set_color('blue')

    for i, label in enumerate(agent_labels):
        if i < len(benchmark_start_states):
            start_x, start_y = benchmark_start_states[i]
            label.set_position((start_x, start_y))
            label.set_text(str(i))
        else:
            label.set_text('')

    time_text.set_text('Time: 0.00')
    return agents + agent_lines + agent_labels + [time_text]


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
        current_angle = 0  # default angle

        for j in range(len(path) - 1):
            # 시간 인덱스를 3으로 수정
            if path[j][3] <= current_time < path[j + 1][3]:
                start = path[j][:2]  # x, y
                start_angle = path[j][2]  # angle
                end_angle = path[j + 1][2]  # angle

                control = agent_controls[j + 1] if j + 1 < len(agent_controls) else (0, 0, 0, 0, 0, 0)
                acc_x, acc_y, t1_x, t1_y, t2_x, t2_y = control

                if all(v == 0 for v in control):
                    # If all control values are zero, keep the agent at its current position
                    current_x, current_y = start
                    current_angle = start_angle
                else:
                    elapsed_time = current_time - path[j][3]
                    total_segment_time = path[j + 1][3] - path[j][3]
                    current_x, current_y = calculate_state(start, (t1_x, t1_y), (acc_x, acc_y), elapsed_time)

                    # Interpolate angle
                    if total_segment_time > 0:
                        angle_progress = elapsed_time / total_segment_time
                        current_angle = start_angle + (end_angle - start_angle) * angle_progress
                    else:
                        current_angle = start_angle

                agents[i].center = (current_x, current_y)
                agent_labels[i].set_text(str(i))
                agent_labels[i].set_position((current_x, current_y))

                # Update orientation line
                line_length = radius * 1.5
                line_x = current_x + line_length * np.cos(current_angle)
                line_y = current_y + line_length * np.sin(current_angle)
                agent_lines[i].set_data([current_x, line_x], [current_y, line_y])
                break
        else:
            # 경로 끝에 도달한 경우
            current_x, current_y = path[-1][:2]
            current_angle = path[-1][2]

            agents[i].center = (current_x, current_y)
            agent_labels[i].set_text(str(i))
            agent_labels[i].set_position((current_x, current_y))

            # Update orientation line
            line_length = radius * 1.5
            line_x = current_x + line_length * np.cos(current_angle)
            line_y = current_y + line_length * np.sin(current_angle)
            agent_lines[i].set_data([current_x, line_x], [current_y, line_y])


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

    # Reset agent colors to blue
    for agent in agents:
        agent.set_color('blue')

    update_agents_positions(current_time)
    detect_collisions(current_time)

    return agents + agent_lines + agent_labels + [time_text]


# Calculate the speed multiplier to keep the animation speed consistent
animation_interval = interval * 10

# 디버깅을 위한 정보 출력
print(f"Total frames: {num_frames}")
print(f"Max time: {max_time}")
print(f"Animation interval: {animation_interval}")

ani = animation.FuncAnimation(fig, update, frames=num_frames, init_func=init, blit=True, interval=animation_interval)

# Set up the writer
Writer = animation.writers['ffmpeg']
writer = Writer(fps=30, metadata=dict(artist='Me'), bitrate=1800)

progress_bar = tqdm(total=num_frames, unit='frames')

def progress_callback(current_frame, total_frames):
    progress_bar.n = current_frame
    progress_bar.refresh()

# Save the animation
# output_file = f"mapf_dynamics.mp4"
# ani.save(output_file, writer=writer, progress_callback=progress_callback)

progress_bar.close()

# print(f"Animation saved as {output_file}")

plt.show()