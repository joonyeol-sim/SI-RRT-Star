import yaml
import random
import numpy as np
import argparse
import os
from shapely.geometry import Point as ShapelyPoint, Polygon
from shapely.ops import unary_union
import math

class Benchmark2DGenerator:
    def __init__(self, width=40, height=40, agent_num=10, obstacle_density=15, robot_radius=0.5):
        """
        2D 벤치마크 생성기 초기화

        Args:
            width: 환경의 너비
            height: 환경의 높이
            agent_num: 에이전트 수
            obstacle_density: 장애물이 차지하는 공간 비율 (%)
            robot_radius: 로봇의 반지름
        """
        self.width = width
        self.height = height
        self.agent_num = agent_num
        self.obstacle_density = obstacle_density  # 퍼센트
        self.robot_radius = robot_radius
        self.obstacles = []
        self.start_points = []
        self.goal_points = []
        self.obstacle_shapes = []  # shapely 객체들 저장

        # 통일된 안전 거리 설정
        self.safe_margin = robot_radius * 2.0  # 통일된 안전 거리

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
        max_attempts = 1500  # 시도 횟수 증가
        attempts = 0

        print(f"목표 장애물 면적: {target_area:.2f} (전체 면적의 {self.obstacle_density}%)")

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
            if temp_area <= target_area * 1.05:  # 5% 여유로 줄임
                self.obstacles.append(obstacle)
                self.obstacle_shapes.append(circle)
                current_area = temp_area

                if len(self.obstacles) % 3 == 0:
                    current_density = self.get_current_obstacle_density()
                    print(f"  장애물 {len(self.obstacles)}개 생성됨 - 현재 밀도: {current_density:.1f}%")

            attempts += 1

        final_density = self.get_current_obstacle_density()
        print(f"✅ 장애물 생성 완료: {len(self.obstacles)}개, 실제 밀도: {final_density:.1f}%")

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

    def generate_valid_point(self, existing_points=None, min_distance=None, exclude_points=None):
        """유효한 2D 점 생성 (기존 점들과 겹치지 않게)"""
        if existing_points is None:
            existing_points = []
        if exclude_points is None:
            exclude_points = []
        if min_distance is None:
            min_distance = self.safe_margin

        max_attempts = 5000  # 시도 횟수 대폭 증가
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
        print(f"⚠️  경고: 첫 번째 시도 실패. 조건 완화하여 재시도...")

        # 2단계: 거리 조건 완화
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

            print(f"   조건 완화 성공 (거리: {relaxed_distance:.2f})")
            return point

        # 3단계: 최소한의 조건만 유지
        print(f"⚠️  최종 완화된 조건으로 생성...")
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
        """시작점과 목표점 생성 (각각 충분한 거리 확보)"""
        self.start_points = []
        self.goal_points = []

        print("시작점 생성 중...")
        print(f"  통일된 최소 거리: {self.safe_margin:.2f}")

        # 시작점 생성
        for i in range(self.agent_num):
            start_point = self.generate_valid_point(
                existing_points=self.start_points,
                min_distance=self.safe_margin
            )
            self.start_points.append(start_point)

            if (i + 1) % 5 == 0 or i == self.agent_num - 1:
                print(f"  시작점 {i + 1}/{self.agent_num} 완료")

        print("목표점 생성 중...")
        print(f"  시작점과의 최소 거리: {self.safe_margin:.2f}")

        # 목표점 생성
        for i in range(self.agent_num):
            start_point = self.start_points[i]

            goal_point = self.generate_valid_point(
                existing_points=self.goal_points,
                min_distance=self.safe_margin,
                exclude_points=[start_point]  # 해당 시작점과의 거리 체크
            )
            self.goal_points.append(goal_point)

            if (i + 1) % 5 == 0 or i == self.agent_num - 1:
                print(f"  목표점 {i + 1}/{self.agent_num} 완료")

    def generate_benchmark(self):
        """전체 벤치마크 생성"""
        print("장애물 생성 중...")
        self.generate_obstacles()

        print("에이전트 점 생성 중...")
        self.generate_agent_points()

        benchmark_data = {
            'dimension': 2,
            'width': self.width,
            'height': self.height,
            'agentNum': self.agent_num,
            'robotRadius': self.robot_radius,
            'obstaclesDensity': round(self.get_current_obstacle_density(), 2),
            'startPoints': self.start_points,
            'goalPoints': self.goal_points,
            'obstacles': self.obstacles
        }

        return benchmark_data

    def save_to_file(self, filename):
        """YAML 파일로 저장"""
        benchmark_data = self.generate_benchmark()

        with open(filename, 'w') as file:
            yaml.dump(benchmark_data, file, default_flow_style=False, indent=2)

        actual_density = self.get_current_obstacle_density()
        print(f"✅ 2D 벤치마크가 {filename}에 저장되었습니다.")
        print(f"   환경 크기: {self.width} x {self.height}")
        print(f"   에이전트 수: {self.agent_num}")
        print(f"   로봇 반지름: {self.robot_radius}")
        print(f"   장애물 수: {len(self.obstacles)}")
        print(f"   장애물 밀도: {actual_density:.1f}% (목표: {self.obstacle_density}%)")

        # 점들 간 거리 검증
        self.validate_points()

    def validate_points(self):
        """생성된 점들의 유효성 검증"""
        print("점 검증 중...")

        # 시작점들 간 거리 확인
        min_start_distance = float('inf')
        for i in range(len(self.start_points)):
            for j in range(i+1, len(self.start_points)):
                dist = np.sqrt((self.start_points[i][0] - self.start_points[j][0])**2 +
                               (self.start_points[i][1] - self.start_points[j][1])**2)
                min_start_distance = min(min_start_distance, dist)

        # 목표점들 간 거리 확인
        min_goal_distance = float('inf')
        for i in range(len(self.goal_points)):
            for j in range(i+1, len(self.goal_points)):
                dist = np.sqrt((self.goal_points[i][0] - self.goal_points[j][0])**2 +
                               (self.goal_points[i][1] - self.goal_points[j][1])**2)
                min_goal_distance = min(min_goal_distance, dist)

        # 시작점-목표점 간 거리 확인
        min_start_goal_distance = float('inf')
        for i in range(len(self.start_points)):
            dist = np.sqrt((self.start_points[i][0] - self.goal_points[i][0])**2 +
                           (self.start_points[i][1] - self.goal_points[i][1])**2)
            min_start_goal_distance = min(min_start_goal_distance, dist)

        print(f"   시작점들 간 최소 거리: {min_start_distance:.3f} (요구: {self.safe_margin:.3f})")
        print(f"   목표점들 간 최소 거리: {min_goal_distance:.3f} (요구: {self.safe_margin:.3f})")
        print(f"   시작-목표 최소 거리: {min_start_goal_distance:.3f} (요구: {self.safe_margin:.3f})")

        # 경고 표시
        if min_start_distance < self.safe_margin:
            print(f"   ⚠️  시작점 간 거리 부족!")
        if min_goal_distance < self.safe_margin:
            print(f"   ⚠️  목표점 간 거리 부족!")
        if min_start_goal_distance < self.safe_margin:
            print(f"   ⚠️  시작-목표점 간 거리 부족!")

def main():
    parser = argparse.ArgumentParser(description='2D Multi-Robot Path Planning 벤치마크 생성기')
    parser.add_argument('--width', type=float, default=40.0, help='환경 너비 (기본값: 40)')
    parser.add_argument('--height', type=float, default=40.0, help='환경 높이 (기본값: 40)')
    parser.add_argument('--agents', type=int, default=10, help='에이전트 수 (기본값: 10)')
    parser.add_argument('--obstacles', type=float, default=15.0, help='장애물 밀도 (%%) (기본값: 15)')
    parser.add_argument('--robot-radius', type=float, default=0.5, help='로봇 반지름 (기본값: 0.5)')
    parser.add_argument('--count', type=int, default=50, help='생성할 벤치마크 파일 수 (기본값: 50)')
    parser.add_argument('--env-name', type=str, default='CircleEnv', help='환경 이름 (기본값: CircleEnv)')

    args = parser.parse_args()

    # 출력 디렉토리 생성
    output_dir = f"benchmark/{args.env_name}_{int(args.obstacles)}"
    agent_dir = os.path.join(output_dir, f"agents{args.agents}")
    os.makedirs(agent_dir, exist_ok=True)

    print(f"🚀 2D 벤치마크 생성 시작...")
    print(f"📁 출력 디렉토리: {agent_dir}")
    print(f"📊 생성할 파일 수: {args.count}")
    print(f"🤖 에이전트 수: {args.agents}")
    print(f"🔴 로봇 반지름: {args.robot_radius}")
    print(f"🏗️  장애물 밀도: {args.obstacles}%")
    print(f"📦 환경 크기: {args.width} x {args.height}")
    print(f"📏 안전 거리: {args.robot_radius * 2.0:.1f}")
    print("=" * 50)

    for i in range(args.count):
        print(f"\n📝 파일 {i+1}/{args.count} 생성 중...")

        generator = Benchmark2DGenerator(
            width=args.width,
            height=args.height,
            agent_num=args.agents,
            obstacle_density=args.obstacles,
            robot_radius=args.robot_radius
        )

        filename = os.path.join(agent_dir, f"{args.env_name}_{int(args.obstacles)}_{args.agents}_{i}.yaml")
        generator.save_to_file(filename)

    print("\n" + "=" * 50)
    print(f"🎉 총 {args.count}개의 2D 벤치마크 파일이 성공적으로 생성되었습니다!")
    print(f"📁 저장 경로: {agent_dir}")
    print(f"📋 파일 형식: {args.env_name}_{int(args.obstacles)}_{args.agents}_[0-{args.count-1}].yaml")

if __name__ == "__main__":
    main()