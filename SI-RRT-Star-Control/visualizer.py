import argparse

import matplotlib.animation as animation
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import yaml

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
    "--interval", "-i", type=float, default=0.01, help="Time interval for updates"
)
args = parser.parse_args()

mapname = args.mapname
obs = args.obs
robotnum = args.robotnum
testnum = args.testnum
interval = args.interval

benchmarkPath = f"benchmark/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}.yaml"
solutionPath = f"solution/{mapname}_{obs}/agents{robotnum}/{mapname}_{obs}_{robotnum}_{testnum}_solution.yaml"

# -- YAML 로드 --
with open(benchmarkPath, "r") as f:
    data = yaml.safe_load(f)

with open(solutionPath, "r") as f:
    sol_data = yaml.safe_load(f)

# 에이전트 수
agent_num = data.get("agentNum", len(data["startPoints"]))

start_points = data.get("startPoints", [])
goal_points = data.get("goalPoints", [])
obstacles = data.get("obstacles", [])

# sol_data의 구조는
# {
#   "solution": [
#       {
#          "agent0": [
#             {
#               "point": {"x":..., "y":...},
#               "velocity": {"x":..., "y":...},
#               "time": ...,
#               "x_control": {
#                   "control_first": {"a":..., "t":...},
#                   "control_const": {"a":..., "t":...},
#                   "control_last":  {"a":..., "t":...}
#               },
#               "y_control": {...}
#             },
#             ... (다음 구간)
#          ]
#       },
#       {
#          "agent1": [...]
#       },
#       ...
#   ]
# }
#
# 이런 형태이므로, 편하게 접근하기 위해 agent별로 파싱하는 과정을 만든다.


##############################################################################
# 1D 가속도/속도/위치 적분을 위한 헬퍼 함수
##############################################################################
def integrate_1d(a1, t1, a2, t2, a3, t3, v0, local_t):
    """
    세 구간(piecewise)로 나누어진 가속도:
      1) a1 로 t1만큼
      2) a2 로 t2만큼
      3) a3 로 t3만큼
    총 시간은 t1 + t2 + t3
    시작 속도는 v0.

    local_t(0 이상)를 줬을 때,
    - local_t가 3개 구간 중 어디까지 왔는지 계산하여
      그때까지의 변위(displacement)와 최종 속도(final velocity)를 반환한다.
    """

    # 처음 상태
    displacement = 0.0
    current_v = v0
    remain_t = local_t

    # 구간 1 (가속도 a1, 시간 t1)
    dt = min(remain_t, t1)
    # s = v0 * dt + 0.5 * a1 * dt^2
    displacement_1 = current_v * dt + 0.5 * a1 * (dt**2)
    displacement += displacement_1
    # v = v0 + a1*dt
    current_v += a1 * dt
    remain_t -= dt

    # 구간 2 (가속도 a2, 시간 t2)
    dt = min(remain_t, t2)
    displacement_2 = current_v * dt + 0.5 * a2 * (dt**2)
    displacement += displacement_2
    current_v += a2 * dt
    remain_t -= dt

    # 구간 3 (가속도 a3, 시간 t3)
    dt = min(remain_t, t3)
    displacement_3 = current_v * dt + 0.5 * a3 * (dt**2)
    displacement += displacement_3
    current_v += a3 * dt
    remain_t -= dt

    return displacement, current_v


##############################################################################
# 각 에이전트의 솔루션(segment) 정보를 구조화
##############################################################################
# 구조 예시:
# agent_solutions[i] = [
#   {
#     "start_time": 0.0,
#     "end_time":   5.2,
#     "start_pos":  (x0, y0),
#     "start_vel":  (vx0, vy0),
#     "x_ctrl": [(a1, t1), (a2, t2), (a3, t3)],
#     "y_ctrl": [(a1, t1), (a2, t2), (a3, t3)]
#   },
#   {
#     ... # 다음 segment
#   }
# ]
##############################################################################

agent_solutions = [[] for _ in range(agent_num)]
max_time = 0.0

# sol_data["solution"]는 agent0, agent1... 순서대로 들어있다.
solution_list = sol_data["solution"]  # list of dicts

# 예: solution_list[0] = {"agent0": [ {...}, {...} ] }
#     solution_list[1] = {"agent1": [ {...}, {...} ] }
# ...
# 따라서 enumerate로 agent index를 뽑아내서 처리한다.
for agent_index, agent_dict_wrapper in enumerate(solution_list):
    # agent_dict_wrapper는 {"agent0": [...]} 이런 식이므로
    # key를 꺼내자 (예: "agent0")
    key = list(agent_dict_wrapper.keys())[0]
    # 그 값이 실제 segment 리스트
    segments = agent_dict_wrapper[key]  # list of points/controls

    # segments가 예: [
    #   {point:..., velocity:..., time:..., x_control:..., y_control:...},
    #   {point:..., velocity:..., time:..., x_control:..., y_control:...}, ...
    # ]
    # 이 구간들을 순서대로 훑으며 segment화
    for i in range(len(segments) - 1):
        curr = segments[i]
        nxt = segments[i + 1]
        start_t = curr["time"]
        end_t = nxt["time"]
        # start pos
        sx = curr["point"]["x"]
        sy = curr["point"]["y"]
        # start vel
        vx0 = curr["velocity"]["x"]
        vy0 = curr["velocity"]["y"]

        # x_control / y_control 은 dict
        #   {control_first: {a, t}, control_const: {a, t}, control_last: {a, t}}
        # 를 각각 꺼내서 tuple로 저장
        def parse_ctrl(ctrl_dict):
            return [
                (ctrl_dict["control_first"]["a"], ctrl_dict["control_first"]["t"]),
                (ctrl_dict["control_const"]["a"], ctrl_dict["control_const"]["t"]),
                (ctrl_dict["control_last"]["a"], ctrl_dict["control_last"]["t"]),
            ]

        x_ctrl = parse_ctrl(nxt["x_control"])
        y_ctrl = parse_ctrl(nxt["y_control"])

        segment_info = {
            "start_time": start_t,
            "end_time": end_t,
            "start_pos": (sx, sy),
            "start_vel": (vx0, vy0),
            "x_ctrl": x_ctrl,  # [(a1,t1),(a2,t2),(a3,t3)]
            "y_ctrl": y_ctrl,
        }
        agent_solutions[agent_index].append(segment_info)

        if end_t > max_time:
            max_time = end_t

    # 마지막 점(segments[-1])도 end_time/velocity 등을 담고 있긴 하나,
    # "다음" segment가 없으면 거기서 끝나므로 보통 2개(스타트+목표)인 경우는 segment가 1개만 생긴다.

# agent_solutions에 이제 각 agent별로 segment가 정리됨


##############################################################################
# 시간 T에 대해서, i번째 에이전트의 (x,y) 위치를 리턴하는 함수
##############################################################################
def get_agent_position(agent_index, T):
    """
    agent_solutions[agent_index]를 보고,
    시간 T일 때의 위치 (x,y) 를 piecewise 가속도 적분으로 계산.
    """
    segments = agent_solutions[agent_index]
    if len(segments) == 0:
        # 솔루션 정보가 없다면, startPoints만 있다고 가정하고 그 자리에 있거나,
        # 혹은 (0,0) 리턴하는 식으로 처리
        if 0 <= agent_index < len(start_points):
            return start_points[agent_index]
        return (0.0, 0.0)

    # 어떤 segment에 속하는지 찾는다.
    # segment i가 start_time <= T < end_time 이면 해당 구간
    # 만약 T가 마지막 구간 end_time 보다 크면 그냥 마지막 구간 끝 지점 위치
    for seg in segments:
        st = seg["start_time"]
        et = seg["end_time"]
        if st <= T < et:
            # 이 세그먼트 내에서 local_time = T - st
            local_time = T - st
            return compute_pos_in_segment(seg, local_time)

    # 마지막 segment를 넘어서는 시간이면, 마지막 segment의 끝 위치(목표 위치)
    last_seg = segments[-1]
    if T >= last_seg["end_time"]:
        # end_time 시점 위치를 그대로 반환
        return compute_pos_in_segment(
            last_seg, last_seg["end_time"] - last_seg["start_time"]
        )
    else:
        # T가 모든 segment start_time보다 작으면, 첫 segment start pos 반환
        first_seg = segments[0]
        return first_seg["start_pos"]


def compute_pos_in_segment(segment, local_t):
    """
    segment 정보와, 세그먼트 안에서 경과시간 local_t가 주어졌을 때
    x,y 위치를 piecewise integration으로 구한다.
    """
    (sx, sy) = segment["start_pos"]
    (vx0, vy0) = segment["start_vel"]

    # x direction
    (a1_x, t1_x), (a2_x, t2_x), (a3_x, t3_x) = segment["x_ctrl"]
    disp_x, _ = integrate_1d(a1_x, t1_x, a2_x, t2_x, a3_x, t3_x, vx0, local_t)

    # y direction
    (a1_y, t1_y), (a2_y, t2_y), (a3_y, t3_y) = segment["y_ctrl"]
    disp_y, _ = integrate_1d(a1_y, t1_y, a2_y, t2_y, a3_y, t3_y, vy0, local_t)

    return (sx + disp_x, sy + disp_y)


##############################################################################
# 애니메이션 준비
##############################################################################
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_xlim(0, 40)
ax.set_ylim(0, 40)
ax.set_aspect("equal")

# 로봇(에이전트) 반지름
radius = 0.5
# 에이전트 객체(원형)과 라벨(텍스트) 준비
agents_patches = [
    patches.Circle((0, 0), radius, color="blue", zorder=10) for _ in range(agent_num)
]
agents_labels = [
    ax.text(
        0, 0, f"{i}", color="white", ha="center", va="center", fontsize=7, zorder=11
    )
    for i in range(agent_num)
]
for circle in agents_patches:
    ax.add_patch(circle)

time_text = ax.text(
    0.02, 0.96, "", transform=ax.transAxes, va="top", ha="left", fontsize=10
)

# 장애물 그리기
for obs in obstacles:
    # obs가 circle( {"center":[x,y], "radius":r} ) 인지 rectangle( {"center":[x,y], "width":..., "height":...} ) 인지에 따라 처리
    if "radius" in obs:
        cx, cy = obs["center"]
        r = obs["radius"]
        c_patch = patches.Circle((cx, cy), r, color="gray", alpha=0.6)
        ax.add_patch(c_patch)
    elif "width" in obs and "height" in obs:
        cx, cy = obs["center"]
        w = obs["width"]
        h = obs["height"]
        r_patch = patches.Rectangle(
            (cx - w / 2, cy - h / 2), w, h, color="gray", alpha=0.6
        )
        ax.add_patch(r_patch)

# 시작/목표 위치 표시
for i in range(agent_num):
    if i < len(start_points):
        sx, sy = start_points[i]
        ax.plot(sx, sy, marker="s", color="green", markersize=4, zorder=5)
    if i < len(goal_points):
        gx, gy = goal_points[i]
        ax.plot(gx, gy, marker="*", color="red", markersize=6, zorder=5)
        ax.text(gx, gy, f"G{i}", fontsize=7, color="black", ha="left", va="bottom")


##############################################################################
# 충돌 감지 (단순 원-원 충돌) 함수
##############################################################################
def detect_collisions(current_time):
    # 에이전트들끼리 거리 < 2*radius 이면 충돌
    positions = [agent.center for agent in agents_patches]
    for i in range(agent_num):
        for j in range(i + 1, agent_num):
            p1 = np.array(positions[i])
            p2 = np.array(positions[j])
            dist = np.linalg.norm(p1 - p2)
            if dist < 2 * radius:
                # 충돌!
                agents_patches[i].set_color("red")
                agents_patches[j].set_color("red")
                # print location
                print(
                    f"Collision: agent {i} at {p1} and {j} at {p2} at time {current_time}"
                )


##############################################################################
# 애니메이션 초기화
##############################################################################
def init():
    for circle in agents_patches:
        circle.center = (0, 0)
        circle.set_color("blue")
    for label in agents_labels:
        label.set_position((0, 0))
    time_text.set_text("")
    return agents_patches + agents_labels + [time_text]


##############################################################################
# 매 프레임마다 업데이트
##############################################################################
def update(frame):
    current_time = frame * interval
    time_text.set_text(f"Time: {current_time:.2f}")

    # 각 에이전트 위치 갱신
    for i in range(agent_num):
        px, py = get_agent_position(i, current_time)
        agents_patches[i].center = (px, py)
        agents_labels[i].set_position((px, py))
        # 충돌 색깔 복구
        agents_patches[i].set_color("blue")

    # 충돌 감지 & 표시
    detect_collisions(current_time)

    return agents_patches + agents_labels + [time_text]


##############################################################################
# 실제 애니메이션 실행
##############################################################################
num_frames = int(max_time / interval) + 2  # 약간 여유
ani = animation.FuncAnimation(
    fig,
    update,
    frames=num_frames,
    init_func=init,
    blit=True,
    interval=interval * 10,  # 밀리초(시각적으로 너무 빠르지 않도록 조절)
)

plt.show()
