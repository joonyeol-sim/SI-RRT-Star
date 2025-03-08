import math
import random

import yaml


def distance(p1, p2):
    """유클리드 거리 계산 함수"""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def can_place_obstacle(
    new_center, new_radius, existing_obstacles, min_dist_obs, min_x, max_x, min_y, max_y
):
    """
    장애물(원)이 기존 장애물들과 겹치지 않는지, 그리고 경계 범위를 넘어가지 않는지 검사
    - new_center: (x, y)
    - new_radius: float
    - existing_obstacles: [{'center':[x,y], 'radius':r}, ...]
    - min_dist_obs: 장애물 간 최소 거리
    - (min_x, max_x, min_y, max_y): 좌표 범위
    """
    # 1) 장애물이 범위를 벗어나는지(원 전체가 범위 내에 있는지) 확인
    if not (min_x + new_radius <= new_center[0] <= max_x - new_radius):
        return False
    if not (min_y + new_radius <= new_center[1] <= max_y - new_radius):
        return False

    # 2) 기존 장애물들과의 거리 확인 (중심거리 >= 두 반지름 합 + 최소 거리)
    for obs in existing_obstacles:
        center = obs["center"]
        radius = obs["radius"]
        dist_centers = distance(new_center, center)
        if dist_centers < (new_radius + radius + min_dist_obs):
            return False

    return True


def can_place_point(
    new_point,
    existing_points,
    obstacles,
    min_dist_points,
    min_dist_obstacle,
    min_x,
    max_x,
    min_y,
    max_y,
):
    """
    로봇 시작점 또는 목표점을 기존 점들과 장애물로부터
    일정 거리 이상 떨어지도록 배치 가능한지 검사
    - new_point: (x, y)
    - existing_points: [(x, y), ...] (동일 카테고리 내에서 최소 거리 유지)
    - obstacles: [{'center':[x,y], 'radius':r}, ...] (장애물과 최소 거리 유지)
    - min_dist_points: 같은 종류(시작점-시작점, 목표점-목표점) 간 최소 거리
    - min_dist_obstacle: 점과 장애물 사이의 최소 거리
    - (min_x, max_x, min_y, max_y): 좌표 범위
    """
    x, y = new_point

    # 1) 범위 내에 있는지 확인
    if not (min_x <= x <= max_x and min_y <= y <= max_y):
        return False

    # 2) 기존 점들과의 최소 거리 확인
    for pt in existing_points:
        if distance(new_point, pt) < min_dist_points:
            return False

    # 3) 장애물과의 최소 거리 확인
    for obs in obstacles:
        center = obs["center"]
        radius = obs["radius"]
        # (장애물의 반지름 + 점-장애물 최소 거리) 이상 떨어져야 함
        if distance(new_point, center) < (radius + min_dist_obstacle):
            return False

    return True


def create_random_circle_env_yaml(
    filename="CircleEnv_random.yaml",
    agent_num=10,  # 로봇(시작점/목표점) 개수
    obstacle_num=10,  # 장애물 개수
    x_range=(0, 40),  # X 좌표 범위
    y_range=(0, 40),  # Y 좌표 범위
    min_dist_between_agents=1.0,  # 시작점들 간 최소 거리, 목표점들 간 최소 거리
    min_dist_point_obstacle=1.0,  # 시작/목표점과 장애물 간 최소 거리
    min_dist_obstacles=0.5,  # 장애물들 간 최소 거리 (중심 간 거리 - 반지름 합)
    radius_range=(1.5, 2.0),  # 장애물 반지름 범위
    seed=None,  # 랜덤 시드(재현성)
    max_attempts=10000,  # 각 오브젝트 배치 시도 횟수(무한 루프 방지)
):
    # 재현성을 위해 시드 고정
    if seed is not None:
        random.seed(seed)

    min_x, max_x = x_range
    min_y, max_y = y_range

    obstacles = []
    startPoints = []
    goalPoints = []

    # 장애물 먼저 배치
    for _ in range(obstacle_num):
        placed = False
        for _ in range(max_attempts):
            center_x = random.uniform(min_x, max_x)
            center_y = random.uniform(min_y, max_y)
            radius = random.uniform(radius_range[0], radius_range[1])

            if can_place_obstacle(
                new_center=(center_x, center_y),
                new_radius=radius,
                existing_obstacles=obstacles,
                min_dist_obs=min_dist_obstacles,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
            ):
                obstacles.append({"center": [center_x, center_y], "radius": radius})
                placed = True
                break
        if not placed:
            raise RuntimeError(
                f"장애물을 {max_attempts}번 시도했지만 배치할 수 없습니다. "
                f"환경 범위나 최소 거리 설정을 조정하세요."
            )

    # 로봇 시작점 배치
    for _ in range(agent_num):
        placed = False
        for _ in range(max_attempts):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if can_place_point(
                new_point=(x, y),
                existing_points=startPoints,
                obstacles=obstacles,
                min_dist_points=min_dist_between_agents,
                min_dist_obstacle=min_dist_point_obstacle,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
            ):
                startPoints.append([x, y])
                placed = True
                break
        if not placed:
            raise RuntimeError(
                f"시작점을 {max_attempts}번 시도했지만 배치할 수 없습니다. "
                f"환경 범위나 최소 거리 설정을 조정하세요."
            )

    # 로봇 목표점 배치
    for _ in range(agent_num):
        placed = False
        for _ in range(max_attempts):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            if can_place_point(
                new_point=(x, y),
                existing_points=goalPoints,
                obstacles=obstacles,
                min_dist_points=min_dist_between_agents,
                min_dist_obstacle=min_dist_point_obstacle,
                min_x=min_x,
                max_x=max_x,
                min_y=min_y,
                max_y=max_y,
            ):
                goalPoints.append([x, y])
                placed = True
                break
        if not placed:
            raise RuntimeError(
                f"목표점을 {max_attempts}번 시도했지만 배치할 수 없습니다. "
                f"환경 범위나 최소 거리 설정을 조정하세요."
            )

    # 최종 데이터 구성
    data = {
        "agentNum": agent_num,
        "goalPoints": goalPoints,
        "obstacles": obstacles,
        "startPoints": startPoints,
    }

    # YAML 파일로 저장
    with open(filename, "w") as f:
        yaml.dump(data, f, sort_keys=False)

    print(f"'{filename}' 파일이 생성되었습니다.")


if __name__ == "__main__":
    create_random_circle_env_yaml(
        filename="CircleEnv_random.yaml",
        agent_num=10,
        obstacle_num=10,
        x_range=(0, 40),
        y_range=(0, 40),
        min_dist_between_agents=1.0,
        min_dist_point_obstacle=1.0,
        min_dist_obstacles=0.5,
        radius_range=(1.5, 2.0),
        seed=42,
        max_attempts=10000,
    )
