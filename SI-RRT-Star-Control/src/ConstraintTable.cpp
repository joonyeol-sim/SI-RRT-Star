#include "ConstraintTable.h"

double ConstraintTable::TimeToCollision(const Point &start_point1, double radius1, Velocity &v1,
                                        const Point &start_point2, double radius2, Velocity &v2, double start_time,
                                        double end_time) const {
  // 두 원(로봇)의 충돌 반경 합
  double combined_radius = radius1 + radius2;

  // Point 구조체를 Eigen::Vector2d로 변환
  Eigen::Vector2d p1(start_point1.x, start_point1.y);
  Eigen::Vector2d p2(start_point2.x, start_point2.y);

  // 두 점 사이의 벡터
  Eigen::Vector2d w = p2 - p1;

  // 충돌 여부 판단: 두 원의 중심 간 거리가 (rA + rC)보다 작으면 이미 충돌 상태
  double c = w.squaredNorm() - combined_radius * combined_radius;
  if (c < 0) {
    return start_time; // 이미 충돌 중이면 시작 시간을 반환
  }

  // Velocity 구조체를 Eigen::Vector2d로 변환
  Eigen::Vector2d v1_e(v1.x, v1.y);
  Eigen::Vector2d v2_e(v2.x, v2.y);

  // 두 속도의 차이를 계산
  Eigen::Vector2d diff_v = v1_e - v2_e;
  double a = diff_v.squaredNorm();
  double b = w.dot(diff_v);

  // 이차 방정식의 판별식
  double discr = b * b - a * c;
  if (discr <= 0) {
    // 실근이 없으면 충돌이 발생하지 않는 것으로 판단
    return -1.0;
  }

  // 충돌이 발생하는 시간 (tau) 계산
  double tau = (b - std::sqrt(discr)) / a;

  // 주어진 시간 구간 내에서 충돌이 발생하는지 검사
  if (tau < 0 || tau > end_time) {
    return -1.0;
  }

  return tau;
}

// 선분(에이전트 중심 경로) vs 원(장애물) 충돌 검사
//  - A, B: 선분의 양 끝점
//  - rA: 에이전트의 반지름
//  - C: 원(장애물) 중심
//  - rC: 원(장애물) 반지름
bool checkCollisionSegmentCircle(const Point &A, const Point &B, double rA, const Point &C, double rC) {
  double R = rA + rC; // 두 원의 합
  auto [Ax, Ay] = A;
  auto [Bx, By] = B;
  auto [Cx, Cy] = C;

  double ABx = Bx - Ax;
  double ABy = By - Ay;
  double ACx = Cx - Ax;
  double ACy = Cy - Ay;
  double AB_len2 = ABx * ABx + ABy * ABy;

  // 1) 직선으로 봤을 때 투영 t 계산
  double t = (ACx * ABx + ACy * ABy) / AB_len2;
  // 2) 선분이므로 [0,1] 범위로 clamp
  if (t < 0.0)
    t = 0.0;
  if (t > 1.0)
    t = 1.0;

  // 3) 선분 위 가장 가까운 점 P(t)
  double closestX = Ax + t * ABx;
  double closestY = Ay + t * ABy;

  // 4) 그 점이 장애물의 중심 C와 R 거리 이하인지 확인
  double dx = (Cx - closestX);
  double dy = (Cy - closestY);
  double dist2 = dx * dx + dy * dy;
  return (dist2 <= (R * R));
}

// Liang-Barsky에 쓸 헬퍼 함수
// p, q => 경계를 나타내는 식에서 나온 계수
// [u1, u2] => 현재 선분 파라미터 t의 유효 구간
// 반환값: 만약 clipTest 결과 구간이 소멸하면 false => 충돌X
static bool clipTest(double p, double q, double &u1, double &u2) {
  // p == 0 => 선이 경계에 평행
  // q < 0 => 완전히 경계 밖, etc.
  if (fabs(p) < 1e-12) {
    // 근사적으로 p==0 => 선이 이 경계와 평행
    if (q < 0.0)
      return false; // 완전히 밖
    // else => 경계에 평행 & 내부 => 구간 변화 없음
    return true;
  }

  double r = q / p;
  if (p < 0.0) {
    // 잠재적 진입
    if (r > u2)
      return false;
    else if (r > u1)
      u1 = r;
  } else {
    // p > 0 => 잠재적 이탈
    if (r < u1)
      return false;
    else if (r < u2)
      u2 = r;
  }
  return true;
}

/**
 * @brief 선분(A->B) vs 직사각형(중심 rectCenter, 폭=rectW, 높이=rectH) 충돌 검사
 *        에이전트 반지름 rA를 직사각형 폭/높이에 합쳐 놓아(확장),
 *        Liang-Barsky 알고리즘으로 "t in [0,1] 교차 구간"을 구해 충돌 여부 반환
 */
bool checkCollisionSegmentRect(const Point &A, const Point &B, double rA, const Point &rectCenter, double rectW,
                               double rectH) {
  // 1) 직사각형 확장
  auto [cx, cy] = rectCenter;
  double halfW = rectW / 2.0 + rA;
  double halfH = rectH / 2.0 + rA;

  // AABB의 left, right, top, bottom
  double left = cx - halfW;
  double right = cx + halfW;
  double top = cy - halfH;
  double bottom = cy + halfH;

  // 2) 선분 A->B를 파라미터화
  double Ax = A.x;
  double Ay = A.y;
  double Bx = B.x;
  double By = B.y;

  double dx = Bx - Ax;
  double dy = By - Ay;

  // Liang-Barsky => u1=0, u2=1에서 시작
  double u1 = 0.0, u2 = 1.0;

  // 경계별로 clipTest(p, q, u1, u2)
  //  "x >= left" => dx>0 => p = -dx, q= Ax - left ...
  //  순서: left, right, top, bottom

  // (1) x >= left  =>  Ax + t*dx >= left
  //  => dx*t >= left - Ax
  //  => dx*t - (left - Ax) >= 0
  {
    double p = -dx;
    double q = Ax - left;
    if (!clipTest(p, q, u1, u2))
      return false;
  }

  // (2) x <= right => Ax + t*dx <= right
  {
    double p = dx;
    double q = right - Ax;
    if (!clipTest(p, q, u1, u2))
      return false;
  }

  // (3) y >= top => Ay + t*dy >= top
  {
    double p = -dy;
    double q = Ay - top;
    if (!clipTest(p, q, u1, u2))
      return false;
  }

  // (4) y <= bottom => Ay + t*dy <= bottom
  {
    double p = dy;
    double q = bottom - Ay;
    if (!clipTest(p, q, u1, u2))
      return false;
  }

  // 만약 여기까지 왔다면, [u1, u2]가 (0~1)에서 소멸하지 않음
  // => 선분이 박스와 교차구간이 존재
  // "충돌"로 본다면 true
  if (u2 < u1)
    return false; // 혹시라도 역전이면 충돌 없음
  if (u1 > 1.0 || u2 < 0.0)
    return false; // 교집합이 [0,1] 범위 밖
  return true;
}

bool ConstraintTable::obstacleConstrained(int agent_id, const Point &from_point, const Velocity &from_velocity,
                                          double from_time, double to_time, const Control &x_control,
                                          const Control &y_control, double agent_radius) const {
  // from_time부터 to_time까지 env.dt 간격으로 반복
  for (double t = from_time; t <= to_time; t += env.dt) {
    // 혹시 t가 부동소수점 때문에 to_time을 살짝 넘어갈 수 있으므로 보정
    if (t > to_time) {
      t = to_time;
    }

    // (1) 현재 시점에서의 경과시간
    double elapsed = t - from_time;

    // (2) evaluateControlState()로 x축, y축 위치 계산
    double px, vx;
    std::tie(px, vx) = evaluateControlState(x_control, from_point.x, from_velocity.x, elapsed);

    double py, vy;
    std::tie(py, vy) = evaluateControlState(y_control, from_point.y, from_velocity.y, elapsed);

    if (px < env.radii[agent_id] || px > env.width - env.radii[agent_id] || py < env.radii[agent_id] ||
        py > env.height - env.radii[agent_id]) {
      return true; // 경계 밖으로 나가면 충돌
    }
    Point current_pos(px, py);

    // (3) 모든 장애물에 대해 충돌 판정
    for (const auto &obs : env.obstacles) {
      // obstacle->constrained(...)는 "current_pos가 obstacle에 닿거나 침범하면 true"
      if (obs->constrained(current_pos, agent_radius)) {
        return true; // 충돌 발생
      }
    }

    // 이미 to_time에 도달했다면 검사 종료
    if (t >= to_time - 1e-9) {
      break;
    }
  }

  return false; // 전 구간에서 충돌 없음
}

bool ConstraintTable::targetConstrained(int agent_id, const Point &from_point, const Velocity &from_velocity,
                                        double from_time, double to_time, const Control &x_control,
                                        const Control &y_control, double radius) const {
  // 다른 모든 에이전트(occupied_agent_id)의 "마지막 지점"을 확인
  for (int occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    auto [last_point, last_velocity, last_time] = path_table[occupied_agent_id].back();
    double other_radius = env.radii[occupied_agent_id];
    if (last_time >= to_time)
      continue;

    for (double t = from_time; t <= to_time; t += env.dt) {
      // 3-1) 우리 에이전트 위치 계산
      double elapsed = t - from_time; // from_time 이후로 얼마나 지났나
      double our_x, our_y, our_vx, our_vy;
      // 예시) evaluateControlState( control, init_pos, init_vel, elapsed_time )
      std::tie(our_x, our_vx) = evaluateControlState(x_control, from_point.x, from_velocity.x, elapsed);
      std::tie(our_y, our_vy) = evaluateControlState(y_control, from_point.y, from_velocity.y, elapsed);

      // 3-2) 상대 에이전트 마지막 위치(정지)와 거리 확인
      double dist = calculateDistance(our_x, our_y, last_point.x, last_point.y);
      if (dist < radius + other_radius + env.epsilon) {
        // 충돌 발생!
        return true;
      }
    }
  }

  return false; // 모든 마지막 지점과 충돌하지 않음
}

bool ConstraintTable::pathConstrained(int agent_id, const Point &from_point, Velocity from_velocity, double from_time,
                                      double to_time, Control &x_control, Control &y_control, double radius) const {
  assert(from_time < to_time);
  // 모든 다른 에이전트에 대해 (충돌 여부 검사)
  for (int other_agent_id = 0; other_agent_id < static_cast<int>(path_table.size()); ++other_agent_id) {
    if (other_agent_id == agent_id)
      continue;
    if (path_table[other_agent_id].empty())
      continue;

    double other_radius = env.radii[other_agent_id];
    const auto &other_path = path_table[other_agent_id];
    const auto &other_trajectory = trajectory_table[other_agent_id]; // 각 segment마다 XYControl 정보가 있음

    // 다른 에이전트의 경로에 대해, 인접한 두 점(즉, 하나의 segment)마다 검사
    // (여기서는 other_trajectory의 크기가 other_path.size()-1라고 가정)
    for (size_t i = 0; i < other_path.size() - 1; ++i) {
      Point prev_point, next_point;
      Velocity prev_velocity, next_velocity;
      double prev_time, next_time;
      std::tie(prev_point, prev_velocity, prev_time) = other_path[i];
      std::tie(next_point, next_velocity, next_time) = other_path[i + 1];

      // 검사 구간이 우리 agent의 시간 [from_time, to_time]와 겹치는지 확인
      if (next_time <= from_time)
        continue;
      if (prev_time >= to_time)
        break;

      // 두 에이전트가 동시에 움직이는 시간 구간
      double start_time = std::max(from_time, prev_time);
      double end_time = std::min(to_time, next_time);

      for (double t = start_time; t <= end_time; t += env.dt) {
        // 우리 에이전트 (계획된 control 적용)
        double our_elapsed = t - from_time;
        double our_x, our_y, our_vx, our_vy;
        std::tie(our_x, our_vx) = evaluateControlState(x_control, from_point.x, from_velocity.x, our_elapsed);
        std::tie(our_y, our_vy) = evaluateControlState(y_control, from_point.y, from_velocity.y, our_elapsed);

        // 다른 에이전트
        double other_elapsed = t - prev_time;
        double other_x, other_y, other_vx, other_vy;
        std::tie(other_x, other_vx) =
            evaluateControlState(other_trajectory[i + 1].x_control, prev_point.x, prev_velocity.x, other_elapsed);
        std::tie(other_y, other_vy) =
            evaluateControlState(other_trajectory[i + 1].y_control, prev_point.y, prev_velocity.y, other_elapsed);

        // 두 에이전트의 위치 거리 계산
        if (calculateDistance(our_x, our_y, other_x, other_y) < radius + other_radius + env.epsilon) {
          return true;
        }
      }
    }
  }
  return false;
}

bool ConstraintTable::hardConstrained(int agent_id, const Point &from_point, const Point &to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);
  // const double velocity_from2to = calculateDistance(from_point, to_point) / env.edge_moving_time;
  const double velocity_from2to = env.max_velocities[agent_id];
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    for (int i = 0; i < constrained_path.size() - 1; i++) {
      auto [prev_point, prev_velocity, prev_time] = constrained_path[i];
      auto [next_point, next_velocity, next_time] = constrained_path[i + 1];

      // check if temporal constraint is satisfied
      if (next_time <= from_time)
        continue;
      if (prev_time >= to_time)
        break;
      // check if spatial constraint is satisfied
      if (calculateDistance(from_point, prev_point) >= calculateDistance(from_point, to_point) + radius +
                                                           calculateDistance(prev_point, next_point) +
                                                           constrained_radius + env.epsilon)
        continue;

      double start_time = max(from_time, prev_time);
      double end_time = min(to_time, next_time);

      // const auto velocity_prev2next = calculateDistance(prev_point, next_point) / env.edge_moving_time;
      const auto velocity_prev2next = env.max_velocities[agent_id];

      auto curr_time = start_time;
      while (curr_time <= end_time) {
        // get point at start_time
        const auto occupied_moving_time = curr_time - prev_time;
        assert(occupied_moving_time >= 0.0);
        const auto occupied_theta = atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);

        auto occupied_point = prev_point;
        if (occupied_theta != 0.0) {
          occupied_point = Point(prev_point.x + velocity_prev2next * cos(occupied_theta) * occupied_moving_time,
                                 prev_point.y + velocity_prev2next * sin(occupied_theta) * occupied_moving_time);
        }

        // get point2 at start_time
        const auto moving_time = curr_time - from_time;
        assert(moving_time >= 0.0);
        const auto theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);

        auto point = from_point;
        if (theta != 0.0) {
          point = Point(from_point.x + velocity_from2to * cos(theta) * moving_time,
                        from_point.y + velocity_from2to * sin(theta) * moving_time);
        }

        if (calculateDistance(point, occupied_point) < radius + constrained_radius + env.epsilon) {
          return true;
        }

        curr_time += env.check_time_resolution;
      }
    }
  }
  return false;
}

void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point &to_point, double radius,
                                               std::vector<Interval> &safe_intervals) const {
  // 1) 초기값: [0, ∞) 전부 "안전"
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, std::numeric_limits<double>::infinity());

  // 2) 다른 모든 에이전트에 대해 충돌 시간대 확인
  for (int occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    // 자기 자신 / 빈 경로 스킵
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    const auto &other_path = path_table[occupied_agent_id];
    const auto &other_trajectory = trajectory_table[occupied_agent_id];

    // 2-1) 상대 에이전트 경로의 각 segment(prev -> next)에 대해
    for (size_t i = 0; i < other_path.size() - 1; ++i) {
      Point prev_point, next_point;
      Velocity prev_velocity, next_velocity;
      double prev_time, next_time;
      std::tie(prev_point, prev_velocity, prev_time) = other_path[i];
      std::tie(next_point, next_velocity, next_time) = other_path[i + 1];

      // "충돌 on/off" 감지용
      bool is_in_collision = false;
      double collision_start = 0.0;

      // (A) segment 내부를 일정 간격 dt로 나누어 시간 t를 증가시키며 위치 계산
      for (double t = prev_time; t <= next_time + 1e-9; t += env.dt) {
        // segment 끝 시간을 초과하지 않도록 clamp(옵션)
        // (부동소수점 오차 방어용. t가 segment_end_time을 조금 넘을 수 있으므로)
        if (t > next_time) {
          t = next_time;
        }

        double elapsed = t - prev_time;

        // (1) 상대 에이전트 위치 계산
        double other_x, other_vx;
        std::tie(other_x, other_vx) =
            evaluateControlState(other_trajectory[i + 1].x_control, prev_point.x, prev_velocity.x, elapsed);

        double other_y, other_vy;
        std::tie(other_y, other_vy) =
            evaluateControlState(other_trajectory[i + 1].y_control, prev_point.y, prev_velocity.y, elapsed);

        // (2) 충돌 판정
        double dist = calculateDistance(to_point.x, to_point.y, other_x, other_y);
        bool collision_now = (dist < (radius + other_radius + env.epsilon));

        // (3) on/off 충돌 구간 업데이트
        if (!is_in_collision && collision_now) {
          // 새로 충돌 시작
          is_in_collision = true;
          collision_start = t;
        } else if (is_in_collision && !collision_now) {
          // 충돌 끝
          insertCollisionIntervalToSIT(safe_intervals, collision_start, t);
          if (safe_intervals.empty())
            return; // 전부 막힘
          is_in_collision = false;
        }

        // 혹시 t가 segment_end_time에 도달했다면 조기 종료
        if (t >= next_time) {
          break;
        }
      }

      // (B) segment 끝까지 충돌이 이어졌다면 [collision_start, segment_end_time]
      if (is_in_collision) {
        insertCollisionIntervalToSIT(safe_intervals, collision_start, next_time);
        if (safe_intervals.empty())
          return; // 전부 막힘
        is_in_collision = false;
      }
    }

    // 2-2) "마지막 점 이후" 영원히 정지 상태를 가정한 충돌 처리
    const auto &[final_point, final_vel, final_time] = other_path.back();
    double end_dist = calculateDistance(to_point.x, to_point.y, final_point.x, final_point.y);
    if (end_dist < radius + other_radius + env.epsilon) {
      insertCollisionIntervalToSIT(safe_intervals, final_time, std::numeric_limits<double>::infinity());
      if (safe_intervals.empty())
        return;
    }
  }

  // 여기까지 오면 safe_intervals에 유효 구간이 남아 있음
}

void ConstraintTable::getSafeIntervalTable(int agent_id, const Point &to_point, double radius,
                                           vector<Interval> &safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    bool is_safe = true;
    double collision_start_time = 0.0;
    for (int i = 0; i < constrained_path.size() - 1; ++i) {
      auto [prev_point, prev_velocity, prev_time] = constrained_path[i];
      auto [next_point, next_velocity, next_time] = constrained_path[i + 1];

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (is_safe &&
            calculateDistance(to_point, interpolated_points[j]) < radius + constrained_radius + env.epsilon) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (!is_safe &&
                   calculateDistance(to_point, interpolated_points[j]) >= radius + constrained_radius + env.epsilon) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertCollisionIntervalToSIT(safe_intervals, collision_start_time, interpolated_times[j]);
          if (safe_intervals.empty())
            return;
        }
      }
    }

    if (!is_safe) {
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time, get<2>(constrained_path.back()));
      if (safe_intervals.empty())
        return;
    }
  }
}

optional<XYControl> ConstraintTable::getEarliestArrivalControl(int agent_id,                  // 에이전트 ID
                                                               const Point &from_point,       // 출발점
                                                               const Point &to_point,         // 도착점
                                                               const Velocity &from_velocity, // 출발 속도
                                                               const Velocity &to_velocity,   // 도착 속도
                                                               double from_time,              // 출발 시간
                                                               double arrival_lower_bound,    // 도착 시간 하한
                                                               double arrival_upper_bound,    // 도착 시간 상한
                                                               double radius) const {

  // 가속도를 점점 낮추면서 찾기
  // 가속도를 계속 낮추다가 만약 도착 시간이 arrival_upper_bound를 넘어가면 실패로 간주
  // 가속도를 계속 낮추다가 만약 도착 시간이 arrival_lower_bound를 넘어가고 충돌이 없으면 성공
  double acc_decrement = 0.01;
  double current_a_max = env.a_max;
  while (true) {
    auto cost_with_controls =
        calculateControls(from_point, to_point, from_velocity, to_velocity, env.v_max, current_a_max);
    current_a_max -= acc_decrement;
    if (current_a_max < 0.0)
      return nullopt;
    if (!cost_with_controls)
      continue;

    auto [control_x, control_y] = std::move(*cost_with_controls);

    double moving_time =
        control_x->control_first.second + control_x->control_const.second + control_x->control_last.second;

    double arrival_time = from_time + moving_time;

    // Check if the agent can reach the target within the time limit
    if (arrival_time < arrival_lower_bound)
      continue;
    if (arrival_time >= arrival_upper_bound)
      return nullopt;

    // Check if the agent is constrained by obstacles
    if (env.algorithm == "pp") {
      // Check if the agent is constrained by obstacles
      if (obstacleConstrained(agent_id, from_point, from_velocity, from_time, arrival_time, *control_x, *control_y,
                              radius))
        return nullopt;
      // Check if the agent is constrained by other agents' paths
      if (targetConstrained(agent_id, from_point, from_velocity, from_time, arrival_time, *control_x, *control_y,
                            radius))
        return nullopt;
      if (!pathConstrained(agent_id, from_point, from_velocity, from_time, arrival_time, *control_x, *control_y,
                           radius))
        return XYControl(*control_x, *control_y);
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_point, to_point, from_time, arrival_time, radius))
        return XYControl(*control_x, *control_y);
    }
  }
  return nullopt;
}

void ConstraintTable::insertCollisionIntervalToSIT(vector<Interval> &safe_intervals, double t_min, double t_max) const {
  assert(t_min >= 0 && t_min < t_max && !safe_intervals.empty());

  int i = 0;
  while (i < safe_intervals.size()) {
    if (t_min >= safe_intervals[i].second) {
      // Collision interval is after the current safe interval
      ++i;
      continue;
    }
    if (t_max <= safe_intervals[i].first) {
      // Collision interval is before the current safe interval
      break;
    }

    if (t_min <= safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      // Collision interval completely covers the current safe interval
      safe_intervals.erase(safe_intervals.begin() + i);
    } else if (t_min <= safe_intervals[i].first && t_max < safe_intervals[i].second) {
      // Collision interval covers the beginning of the current safe interval
      safe_intervals[i].first = t_max;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      // Collision interval covers the end of the current safe interval
      safe_intervals[i].second = t_min;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max < safe_intervals[i].second) {
      // Collision interval covers the middle of the current safe interval
      safe_intervals.insert(safe_intervals.begin() + i + 1, {t_max, safe_intervals[i].second});
      safe_intervals[i].second = t_min;
      ++i;
    }
  }
}

// bool ConstraintTable::checkConflicts(const PathSolution &solution) const {
//   for (int agent1_id = 0; agent1_id < solution.size(); ++agent1_id) {
//     for (int i = 0; i < solution[agent1_id].size() - 1; ++i) {
//       auto [from_point, from_velocity, from_time] = solution[agent1_id][i];
//       auto [to_point, to_velocity, to_time] = solution[agent1_id][i + 1];
//       vector<Point> interpolated_points;
//       vector<double> interpolated_times;
//       interpolatePointTime(agent1_id, from_point, to_point, from_time, to_time, interpolated_points,
//                            interpolated_times);
//
//       for (auto agent2_id = 0; agent2_id < solution.size(); ++agent2_id) {
//         if (agent1_id == agent2_id)
//           continue;
//         // path conflict
//         for (int j = 0; j < solution[agent2_id].size() - 1; ++j) {
//           auto [prev_point, prev_velocity, prev_time] = solution[agent2_id][j];
//           auto [next_point, next_velocity, next_time] = solution[agent2_id][j + 1];
//
//           // check if temporal constraint is satisfied
//           if (next_time <= from_time)
//             continue;
//           if (prev_time >= to_time)
//             break;
//           // check if spatial constraint is satisfied
//           if (calculateDistance(from_point, prev_point) >=
//               calculateDistance(from_point, to_point) + env.radii[agent1_id] +
//                   calculateDistance(prev_point, next_point) + env.radii[agent2_id] + env.epsilon)
//             continue;
//
//           double start_time = max(from_time, prev_time);
//           double end_time = min(to_time, next_time);
//
//           auto curr_time = start_time;
//           while (curr_time <= end_time) {
//             // get point at start_time
//             const auto occupied_moving_time = curr_time - prev_time;
//             assert(occupied_moving_time >= 0.0);
//             const auto occupied_theta = atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
//
//             auto occupied_point = prev_point;
//             if (occupied_theta != 0.0) {
//               occupied_point =
//                   Point(prev_point.x + env.max_velocities[agent2_id] * cos(occupied_theta) * occupied_moving_time,
//                         prev_point.y + env.max_velocities[agent2_id] * sin(occupied_theta) * occupied_moving_time);
//             }
//
//             // get point2 at start_time
//             const auto moving_time = curr_time - from_time;
//             assert(moving_time >= 0.0);
//             const auto theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);
//
//             auto point = from_point;
//             if (theta != 0.0) {
//               point = Point(from_point.x + env.max_velocities[agent1_id] * cos(theta) * moving_time,
//                             from_point.y + env.max_velocities[agent1_id] * sin(theta) * moving_time);
//             }
//
//             if (calculateDistance(point, occupied_point) < env.radii[agent1_id] + env.radii[agent2_id]) {
//               // cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time " <<
//               curr_time
//               //      << endl;
//               // cout << "From: (" << get<0>(from_point) << ", " << get<1>(from_point) << "), t: " << from_time <<
//               endl;
//               // cout << "To: (" << get<0>(to_point) << ", " << get<1>(to_point) << "), t: " << to_time << endl;
//               // cout << "FTPoint: (" << get<0>(point) << ", " << get<1>(point) << ")" << endl;
//               // cout << "Prev: (" << get<0>(prev_point) << ", " << get<1>(prev_point) << "), t: " << prev_time <<
//               endl;
//               // cout << "Next: (" << get<0>(next_point) << ", " << get<1>(next_point) << "), t: " << next_time <<
//               endl;
//               // cout << "PNPoint: (" << get<0>(occupied_point) << ", " << get<1>(occupied_point) << ")" << endl;
//               // cout << "Distance: " << calculateDistance(point, occupied_point) << endl;
//               return true;
//             }
//
//             curr_time += env.check_time_resolution;
//           }
//         }
//
//         // target conflict
//         auto [last_point, last_velocity, last_time] = solution[agent2_id].back();
//         if (last_time >= to_time)
//           continue;
//         if (calculateDistance(from_point, last_point) >=
//             env.radii[agent1_id] + calculateDistance(from_point, to_point) + env.radii[agent2_id] + env.epsilon)
//           continue;
//         for (int j = 0; j < interpolated_points.size(); ++j) {
//           if (last_time >= interpolated_times[j])
//             continue;
//           if (calculateDistance(last_point, interpolated_points[j]) < env.radii[agent1_id] + env.radii[agent2_id])
//           {
//             // cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time "
//             //      << interpolated_times[j] << endl;
//             // cout << "Point: (" << get<0>(interpolated_points[j]) << ", " << get<1>(interpolated_points[j]) <<
//             ")"
//             //      << endl;
//             // cout << "Last Point: (" << get<0>(last_point) << ", " << get<1>(last_point) << ")" << endl;
//             return true;
//           }
//         }
//       }
//     }
//   }
//   return false;
// }
