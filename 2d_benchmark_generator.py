import yaml
import random
import numpy as np
import argparse
import os
from shapely.geometry import Point as ShapelyPoint, Polygon
from shapely.ops import unary_union
import math

class Benchmark2DGenerator:
    def __init__(self, width=40, height=40, agent_num=10, obstacle_density=15, robot_radius=0.5, use_random_angle=True):
        """
        2D 벤치마크 생성기 초기화

        Args:
            width: 환경의 너비
            height: 환경의 높이
            agent_num: 에이전트 수
            obstacle_density: 장애물이 차지하는 공간 비율 (%)
            robot_radius: 로봇의 반지름
            use_random_angle: 로봇의 랜덤 초기 각도 사용 여부
        """
        self.width = width
        self.height = height
        self.agent_num = agent_num
        self.obstacle_density = obstacle_density  # 퍼센트
        self.robot_radius = robot_radius
        self.use_random_angle = use_random_angle
        self.obstacles = []
        self.agents = []  # 에이전트별 정보를 저장할 리스트
        self.obstacle_shapes = []  # shapely 객체들 저장

        # 통일된 안전 거리 설정
        self.safe_margin = robot_radius * 2.0

    def generate_random_angle(self):
        """0도에서 360도 사이의 랜덤 각도 생성 (라디안)"""
        return random.uniform(0, 2 * math.pi)

    def normalize_angle(self, angle):
        """각도를 0~2π 범위로 정규화"""
        while angle < 0:
            angle += 2 * math.pi
        while angle >= 2 * math.pi:
            angle -= 2 * math.pi
        return angle

    def angle_to_degrees(self, angle_rad):
        """라디안을 도로 변환"""
        return math.degrees(angle_rad)

    def generate_circle_obstacle(self):
        """2D 원형 장애물 생성"""
        # 장애물을 위한 안전 여백 고려
        margin = 3.0  # 장애물이 경계에서 떨어진 거리
        center_x = random.uniform(margin, self.width - margin)
        center_y = random.uniform(margin, self.height - margin)
        radius = random.uniform(1.0, 2.5)  # 장애물 크기 약간 줄임

        # 경계를 벗어나지 않도록 조정
        center_x = max(radius + margin, min(self.width - radius - margin, center_x))
        center_y = max(radius + margin, min(self.height - radius - margin, center_y))

        return {
            'center': [center_x, center_y],
            'radius': radius
        }

    def calculate_total_obstacle_area(self):
        """현재 장애물들의 총 면적 계산 (겹치는 부분 제외)"""
        if not self.obstacle_shapes:
            return 0.0

        # 모든 장애물을 합집합으로 계산 (겹치는 부분 제거)
        union = unary_union(self.obstacle_shapes)
        return union.area

    def get_current_obstacle_density(self):
        """현재 장애물 밀도 계산 (%)"""
        total_area = self.width * self.height
        obstacle_area = self.calculate_total_obstacle_area()
        return (obstacle_area / total_area) * 100

    def generate_obstacles(self):
        """목표 밀도에 맞춰 장애물 생성"""
        self.obstacles = []
        self.obstacle_shapes = []

        target_area = (self.width * self.height) * (self.obstacle_density / 100.0)
        current_area = 0.0
        max_attempts = 1500
        attempts = 0

        while current_area < target_area and attempts < max_attempts:
            obstacle = self.generate_circle_obstacle()

            # shapely 원형 객체 생성
            center = obstacle['center']
            radius = obstacle['radius']
            circle = ShapelyPoint(center[0], center[1]).buffer(radius)

            # 환경 경계 내에 있는지 확인
            env_boundary = Polygon([(0, 0), (self.width, 0), (self.width, self.height), (0, self.height)])
            if not env_boundary.contains(circle):
                attempts += 1
                continue

            # 임시로 추가해서 면적 계산
            temp_shapes = self.obstacle_shapes + [circle]
            temp_area = unary_union(temp_shapes).area if temp_shapes else 0

            # 목표 면적을 초과하지 않으면 추가
            if temp_area <= target_area * 1.05:
                self.obstacles.append(obstacle)
                self.obstacle_shapes.append(circle)
                current_area = temp_area

            attempts += 1

    def is_point_valid_for_robot(self, point):
        """점이 장애물과 충돌하지 않는지 확인 (로봇 반지름 + 안전거리 고려)"""
        x, y = point

        # 경계 확인 (더 넉넉한 여백)
        boundary_margin = self.robot_radius + 1.0
        if x < boundary_margin or x > self.width - boundary_margin:
            return False
        if y < boundary_margin or y > self.height - boundary_margin:
            return False

        # 로봇을 나타내는 원 생성 (안전 거리 포함)
        total_safe_distance = self.robot_radius + self.safe_margin
        robot_circle = ShapelyPoint(x, y).buffer(total_safe_distance)

        # 모든 장애물과의 충돌 확인
        for obstacle_shape in self.obstacle_shapes:
            if robot_circle.intersects(obstacle_shape):
                return False

        return True

    def check_point_collision_with_existing(self, point, existing_points, min_distance):
        """새 점이 기존 점들과 최소 거리를 유지하는지 확인"""
        x, y = point

        for existing_point in existing_points:
            ex, ey = existing_point
            distance = np.sqrt((x - ex)**2 + (y - ey)**2)
            if distance < min_distance:
                return False
        return True

    def get_existing_points(self):
        """현재까지 생성된 모든 점들 반환"""
        points = []
        for agent in self.agents:
            if 'startState' in agent:
                points.append(agent['startState']['position'])
            if 'goalState' in agent:
                points.append(agent['goalState']['position'])
        return points

    def generate_valid_point(self, existing_points=None, min_distance=None, exclude_points=None):
        """유효한 2D 점 생성 (기존 점들과 겹치지 않게)"""
        if existing_points is None:
            existing_points = []
        if exclude_points is None:
            exclude_points = []
        if min_distance is None:
            min_distance = self.safe_margin

        max_attempts = 5000
        boundary_margin = self.robot_radius + 1.0

        for attempt in range(max_attempts):
            x = random.uniform(boundary_margin, self.width - boundary_margin)
            y = random.uniform(boundary_margin, self.height - boundary_margin)
            point = [x, y]

            # 장애물과의 충돌 확인
            if not self.is_point_valid_for_robot(point):
                continue

            # 기존 점들과의 거리 확인
            if not self.check_point_collision_with_existing(point, existing_points, min_distance):
                continue

            # 제외할 점들과의 거리 확인 (예: 해당 에이전트의 시작점)
            if exclude_points and not self.check_point_collision_with_existing(point, exclude_points, self.safe_margin):
                continue

            return point

        # 실패시 조건을 단계적으로 완화
        relaxed_distance = min_distance * 0.7
        for attempt in range(max_attempts // 2):
            x = random.uniform(boundary_margin, self.width - boundary_margin)
            y = random.uniform(boundary_margin, self.height - boundary_margin)
            point = [x, y]

            if not self.is_point_valid_for_robot(point):
                continue
            if not self.check_point_collision_with_existing(point, existing_points, relaxed_distance):
                continue
            if exclude_points and not self.check_point_collision_with_existing(point, exclude_points, self.safe_margin * 0.7):
                continue

            return point

        # 최소한의 조건만 유지
        for attempt in range(max_attempts // 4):
            x = random.uniform(boundary_margin, self.width - boundary_margin)
            y = random.uniform(boundary_margin, self.height - boundary_margin)
            point = [x, y]

            if self.is_point_valid_for_robot(point):
                return point

        # 최후의 수단
        return [
            random.uniform(boundary_margin, self.width - boundary_margin),
            random.uniform(boundary_margin, self.height - boundary_margin)
        ]

    def generate_agent_points(self):
        """에이전트별 시작/목표 상태 생성"""
        self.agents = []

        for i in range(self.agent_num):
            # 기존 점들 수집
            existing_points = self.get_existing_points()

            # 시작점 생성
            start_point = self.generate_valid_point(
                existing_points=existing_points,
                min_distance=self.safe_margin
            )

            # 시작 각도 생성
            start_angle = self.generate_random_angle() if self.use_random_angle else 0.0

            # 시작 상태 생성
            start_state = {
                'position': start_point,
                'angle': round(start_angle, 4)
            }

            # 목표점 생성 (시작점 포함하여 기존 점들과 거리 확인)
            existing_points_with_start = existing_points + [start_point]
            goal_point = self.generate_valid_point(
                existing_points=existing_points_with_start,
                min_distance=self.safe_margin,
                exclude_points=[start_point]
            )

            # 목표 각도 생성
            goal_angle = self.generate_random_angle() if self.use_random_angle else 0.0

            # 목표 상태 생성
            goal_state = {
                'position': goal_point,
                'angle': round(goal_angle, 4)
            }

            # 에이전트 정보 생성
            agent = {
                'id': i,
                'startState': start_state,
                'goalState': goal_state
            }

            self.agents.append(agent)

    def generate_benchmark(self):
        """전체 벤치마크 생성"""
        self.generate_obstacles()
        self.generate_agent_points()

        benchmark_data = {
            'dimension': 2,
            'width': self.width,
            'height': self.height,
            'agentNum': self.agent_num,
            'robotRadius': self.robot_radius,
            'obstaclesDensity': round(self.get_current_obstacle_density(), 2),
            'useRandomAngle': self.use_random_angle,
            'agents': self.agents,
            'obstacles': self.obstacles
        }

        return benchmark_data

    def save_to_file(self, filename):
        """YAML 파일로 저장"""
        benchmark_data = self.generate_benchmark()

        with open(filename, 'w') as file:
            yaml.dump(benchmark_data, file, default_flow_style=False, indent=2)

    def validate_points(self):
        """생성된 점들의 유효성 검증"""
        # 모든 점들 수집
        all_points = []
        for agent in self.agents:
            all_points.append(agent['startState']['position'])
            all_points.append(agent['goalState']['position'])

        # 점들 간 최소 거리 확인
        min_distance = float('inf')
        for i in range(len(all_points)):
            for j in range(i+1, len(all_points)):
                dist = np.sqrt((all_points[i][0] - all_points[j][0])**2 +
                               (all_points[i][1] - all_points[j][1])**2)
                min_distance = min(min_distance, dist)

        return min_distance >= self.safe_margin

def main():
    parser = argparse.ArgumentParser(description='2D Multi-Robot Path Planning 벤치마크 생성기')
    parser.add_argument('--width', type=float, default=40.0, help='환경 너비 (기본값: 40)')
    parser.add_argument('--height', type=float, default=40.0, help='환경 높이 (기본값: 40)')
    parser.add_argument('--agents', type=int, default=10, help='에이전트 수 (기본값: 10)')
    parser.add_argument('--obstacles', type=float, default=15.0, help='장애물 밀도 (%%) (기본값: 15)')
    parser.add_argument('--robot-radius', type=float, default=0.5, help='로봇 반지름 (기본값: 0.5)')
    parser.add_argument('--count', type=int, default=50, help='생성할 벤치마크 파일 수 (기본값: 50)')
    parser.add_argument('--env-name', type=str, default='CircleEnv', help='환경 이름 (기본값: CircleEnv)')
    parser.add_argument('--no-random-angle', action='store_true', help='랜덤 각도 비활성화 (기본값: 활성화)')

    args = parser.parse_args()

    # 출력 디렉토리 생성
    output_dir = f"benchmark/{args.env_name}_{int(args.obstacles)}"
    agent_dir = os.path.join(output_dir, f"agents{args.agents}")
    os.makedirs(agent_dir, exist_ok=True)

    use_random_angle = not args.no_random_angle

    print(f"🚀 2D 벤치마크 생성 시작...")
    print(f"📁 출력 디렉토리: {agent_dir}")
    print(f"📊 파일 수: {args.count}, 에이전트: {args.agents}, 장애물: {args.obstacles}%, 최소 거리: {args.robot_radius * 2.0:.1f}")
    print(f"🔄 랜덤 각도: {'활성화' if use_random_angle else '비활성화'}")

    success_count = 0
    for i in range(args.count):
        generator = Benchmark2DGenerator(
            width=args.width,
            height=args.height,
            agent_num=args.agents,
            obstacle_density=args.obstacles,
            robot_radius=args.robot_radius,
            use_random_angle=use_random_angle
        )

        filename = os.path.join(agent_dir, f"{args.env_name}_{int(args.obstacles)}_{args.agents}_{i}.yaml")
        generator.save_to_file(filename)

        if generator.validate_points():
            success_count += 1

        if (i + 1) % 10 == 0:
            print(f"✅ {i + 1}/{args.count} 완료 (성공률: {success_count}/{i + 1})")

    print(f"🎉 생성 완료! 총 {args.count}개 파일 생성 (성공률: {success_count}/{args.count})")

if __name__ == "__main__":
    main()