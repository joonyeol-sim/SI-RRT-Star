#!/usr/bin/env python3
import argparse

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml


def eval_axis_position(controls, t, init=0.0, v0=0.0):
    """특정 축(x 또는 y)의 제어 명령을 적용하여 위치를 계산"""
    pos = 0.0
    v = v0
    for seg in controls:
        for phase in ["control_first", "control_const", "control_last"]:
            phase_duration = seg[phase]["t"]
            a = seg[phase]["a"]
            if t <= phase_duration:
                pos += v * t + 0.5 * a * t * t
                return pos
            else:
                pos += v * phase_duration + 0.5 * a * phase_duration * phase_duration
                v += a * phase_duration
                t -= phase_duration
    return pos


def get_total_time(controls):
    """한 축의 전체 제어 명령 실행 시간을 계산"""
    return sum(
        seg["control_first"]["t"] + seg["control_const"]["t"] + seg["control_last"]["t"]
        for seg in controls
    )


def compute_agent_trajectory(agent_control, start, dt=0.001):
    """각 에이전트의 궤적을 미리 계산"""
    x_controls = agent_control.get("x_controls", [])
    y_controls = agent_control.get("y_controls", [])
    T = get_total_time(x_controls)  # 전체 제어 시간
    times = np.arange(0, T + dt, dt)
    trajectory = [
        (
            t,
            start[0] + eval_axis_position(x_controls, t, init=0.0, v0=0.0),
            start[1] + eval_axis_position(y_controls, t, init=0.0, v0=0.0),
        )
        for t in times
    ]
    return trajectory


def compute_all_trajectories(control_data, start_points, dt=0.001):
    """모든 에이전트의 궤적을 계산"""
    return [
        compute_agent_trajectory(agent, start_points[i], dt)
        for i, agent in enumerate(control_data.get("control_solution", []))
    ]


# --- 메인 코드 ---
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
parser.add_argument(
    "--speed", "-s", type=float, default=5.0, help="Simulation speed factor"
)  # 기본값 증가
args = parser.parse_args()

mapname, obs, robotnum, testnum, interval, speed_factor = (
    args.mapname,
    args.obs,
    args.robotnum,
    args.testnum,
    args.interval,
    args.speed,
)

# 파일 로드
benchmarkPath = f"benchmark/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}.yaml"
controlPath = f"solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_control.yaml"

with open(benchmarkPath, "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
with open(controlPath, "r") as f:
    control_data = yaml.load(f, Loader=yaml.FullLoader)

obstacles = data.get("obstacles", [])
start_points = data.get("startPoints", [])
goal_points = data.get("goalPoints", [])

# 궤적 계산 (더 작은 dt 사용)
trajectories = compute_all_trajectories(control_data, start_points, dt=0.001)

# 전체 프레임 개수
if trajectories:
    max_time = trajectories[0][-1][0]
    num_frames = len(trajectories[0]) // int(speed_factor)
else:
    max_time, num_frames = 0, 0

# 시각화 설정
fig, ax = plt.subplots(figsize=(8, 8))
radius = 0.5
agents = [
    patches.Circle(tuple(start), radius, color="blue", fill=True)
    for start in start_points
]
agent_labels = [
    ax.text(*start, str(i), fontsize=8, color="white", ha="center", va="center")
    for i, start in enumerate(start_points)
]
time_text = ax.text(
    0.005,
    0.995,
    "",
    transform=ax.transAxes,
    horizontalalignment="left",
    verticalalignment="top",
)

for agent in agents:
    ax.add_patch(agent)

# 장애물 그리기
for obs in obstacles:
    if "radius" in obs:
        ax.add_patch(
            patches.Circle(obs["center"], obs["radius"], color="gray", fill=True)
        )
    elif "width" in obs and "height" in obs:
        ax.add_patch(
            patches.Rectangle(
                (
                    obs["center"][0] - obs["width"] / 2,
                    obs["center"][1] - obs["height"] / 2,
                ),
                obs["width"],
                obs["height"],
                color="gray",
                fill=True,
            )
        )

# 시작점과 목표점
for i, (start, goal) in enumerate(zip(start_points, goal_points)):
    ax.plot(*start, marker="o", markersize=8, color="green")
    ax.plot(*goal, marker="*", markersize=10, color="red")
    ax.text(*goal, f"G{i}", fontsize=8, color="black", ha="right", va="bottom")


def init():
    ax.set_xlim(0, 40)
    ax.set_ylim(0, 40)
    ax.set_aspect("equal")
    for i, agent in enumerate(agents):
        agent.center = tuple(start_points[i])
        agent_labels[i].set_position(tuple(start_points[i]))
    time_text.set_text("")
    return agents + agent_labels + [time_text]


def update(frame):
    frame_index = int(frame * speed_factor * 2)  # 더 빠르게 진행
    current_time = frame_index * interval
    time_text.set_text(f"Time: {current_time:.2f}")

    for i, traj in enumerate(trajectories):
        if frame_index < len(traj):
            _, x, y = traj[frame_index]
        else:
            _, x, y = traj[-1]
        agents[i].center = (x, y)
        agent_labels[i].set_position((x, y))

    return agents + agent_labels + [time_text]


# 애니메이션 속도 향상
animation_interval = 5  # 초당 200 FPS (매우 빠르게)
ani = animation.FuncAnimation(
    fig,
    update,
    frames=num_frames,
    init_func=init,
    blit=True,
    interval=animation_interval,
)

plt.title("Super Fast & Smooth Control-based Trajectory Visualization")
plt.show()
