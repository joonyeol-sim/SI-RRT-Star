#include "ConstraintTable.h"

double ConstraintTable::TimeToCollision(const Point &start_point1, double radius1, Velocity &v1,
                                        const Point &start_point2, double radius2, Velocity &v2, double start_time,
                                        double end_time) const {
  double combined_radius = radius1 + radius2;
  Point w = start_point2 - start_point1;
  double c = w.dot(w) - combined_radius * combined_radius;

  if (c < 0) {
    // Agents are already colliding
    return start_time;
  }

  Velocity difference_v = v1 - v2;
  double a = difference_v.dot(difference_v);
  double b = w.dot(difference_v);
  double discr = b * b - a * c;

  if (discr <= 0) {
    // No collision
    return -1.0;
  }

  double tau = (b - std::sqrt(discr)) / a;

  if (tau < 0 or tau > end_time) {
    // Collision occurs outside the time interval
    return -1.0;
  }

  return tau;
}

// double ConstraintTable::TimeToCollision(const Point &start_point1, double radius1, Velocity &v1,
//                                         const Point &start_point2, double radius2, Velocity &v2, double start_time,
//                                         double end_time) const {
//   double c = (start_point2.x - start_point1.x) * (start_point2.x - start_point1.x) +
//              (start_point2.y - start_point1.y) * (start_point2.y - start_point1.y) -
//              (radius1 + radius2) * (radius1 + radius2);
//
//   if (c < 0) {
//     // Agents are already colliding
//     return start_time;
//   }
//
//   double a = (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y);
//   double b = (start_point2.x - start_point1.x) * (v1.x - v2.x) + (start_point2.y - start_point1.y) * (v1.y - v2.y);
//   double discr = b * b - a * c;
//
//   if (discr <= 0) {
//     // No collision
//     return -1.0;
//   }
//
//   double tau = (b - std::sqrt(discr)) / a;
//
//   if (tau < 0 or tau > end_time) {
//     // Collision occurs outside the time interval
//     return -1.0;
//   }
//
//   return tau;
// }

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

bool ConstraintTable::obstacleConstrained(int agent_id, const Point &from_point, const Point &to_point,
                                          double agent_radius) const {
  // 에이전트가 (from_point -> to_point)로 이동하는 선분을
  // 모든 장애물과 각각 "선분 vs 장애물" 검사
  for (const auto &obs : env.obstacles) {
    // 원형 장애물인 경우
    if (auto *circ = dynamic_cast<CircularObstacle *>(obs.get())) {
      // 선분 vs 원 충돌 검사
      if (checkCollisionSegmentCircle(from_point, to_point, agent_radius, circ->point, circ->radius)) {
        return true; // 충돌
      }
    }
    // 직사각형 장애물인 경우
    else if (auto *rect = dynamic_cast<RectangularObstacle *>(obs.get())) {
      // 선분 vs 직사각형 충돌 검사
      if (checkCollisionSegmentRect(from_point, to_point, agent_radius, rect->point, rect->width, rect->height)) {
        return true; // 충돌
      }
    }
  }
  // 모든 장애물과 충돌X
  return false;
}

bool ConstraintTable::targetConstrained(int agent_id, const Point &from_point, const Point &to_point, double from_time,
                                        double to_time, double radius) const {

  for (auto occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    // target conflict
    auto [last_point, last_time] = path_table[occupied_agent_id].back();
    // check if temporal constraint is satisfied
    if (last_time >= to_time)
      continue;

    if (checkCollisionSegmentCircle(from_point, to_point, radius, last_point, other_radius) == true) {
      return true; // 충돌
    }
  }
  return false;
}

bool ConstraintTable::pathConstrained(int agent_id, const Point &from_point, const Point &to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);

  // Calculate velocity for agent_id
  const auto theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);
  Velocity v1;
  if (theta == 0.0)
    v1 = Velocity(0.0, 0.0);
  else
    v1 = Velocity(env.max_velocities[agent_id] * cos(theta), env.max_velocities[agent_id] * sin(theta));

  for (auto occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    const auto &other_path = path_table[occupied_agent_id];

    // vertex-edge conflict
    for (int i = 0; i < static_cast<int>(other_path.size()) - 1; ++i) {
      auto [prev_point, prev_time] = other_path[i];
      auto [next_point, next_time] = other_path[i + 1];

      // check if temporal constraint is satisfied
      if (next_time <= from_time)
        continue;
      if (prev_time >= to_time)
        break;

      // check if spatial constraint is satisfied
      if (calculateDistance(from_point, prev_point) >= calculateDistance(from_point, to_point) + radius +
                                                           calculateDistance(prev_point, next_point) +
                                                           env.radii[occupied_agent_id] + env.epsilon)
        continue;

      // Calculate velocity for occupied_agent_id
      const auto occupied_theta = atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);
      Velocity v2;
      if (occupied_theta == 0.0)
        v2 = Velocity(0.0, 0.0);
      else
        v2 = Velocity(env.max_velocities[occupied_agent_id] * cos(occupied_theta),
                      env.max_velocities[occupied_agent_id] * sin(occupied_theta));

      double start_time = std::max(from_time, prev_time);
      double end_time = std::min(to_time, next_time);

      Point start_point1 =
          Point(from_point.x + v1.x * (start_time - from_time), from_point.y + v1.y * (start_time - from_time));
      Point start_point2 =
          Point(prev_point.x + v2.x * (start_time - prev_time), prev_point.y + v2.y * (start_time - prev_time));

      double collision_t = TimeToCollision(
          /* agent1 */ start_point1, radius, v1,
          /* agent2 */ start_point2, other_radius, v2,
          /* time */ start_time, end_time);

      if (collision_t > 0.0) {
        return true; // 충돌
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
      auto [prev_point, prev_time] = constrained_path[i];
      auto [next_point, next_time] = constrained_path[i + 1];

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

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point &to_point, double radius,
                                               vector<Interval> &safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;
    // vertex-edge conflict
    bool is_safe = true;
    double collision_start_time = 0.0;
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(occupied_agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (is_safe &&
            calculateDistance(to_point, interpolated_points[j]) < radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (!is_safe && calculateDistance(to_point, interpolated_points[j]) >=
                                   radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertCollisionIntervalToSIT(safe_intervals, collision_start_time, interpolated_times[j]);
          if (safe_intervals.empty())
            return;
        }
      }
    }
    if (!is_safe) {
      // target conflict
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time, numeric_limits<double>::infinity());
      if (safe_intervals.empty())
        return;
    }
  }
}

void ConstraintTable::getSafeIntervalTable(int agent_id, const Point &to_point, double radius,
                                           vector<Interval> &safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    bool is_safe = true;
    double collision_start_time = 0.0;
    for (int i = 0; i < constrained_path.size() - 1; ++i) {
      auto [prev_point, prev_time] = constrained_path[i];
      auto [next_point, next_time] = constrained_path[i + 1];

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
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
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time, get<1>(constrained_path.back()));
      if (safe_intervals.empty())
        return;
    }
  }
}

double ConstraintTable::getEarliestArrivalTime(int agent_id, const Point &from_point, const Point &to_point,
                                               double expand_time, double lower_bound, double upper_bound,
                                               double radius) const {
  double earliest_arrival_time = lower_bound;
  while (earliest_arrival_time < upper_bound) {
    if (env.algorithm == "pp") {
      if (targetConstrained(agent_id, from_point, to_point, earliest_arrival_time - expand_time, earliest_arrival_time,
                            radius))
        return -1.0;
      if (!pathConstrained(agent_id, from_point, to_point, earliest_arrival_time - expand_time, earliest_arrival_time,
                           radius))
        return earliest_arrival_time;
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_point, to_point, earliest_arrival_time - expand_time, earliest_arrival_time,
                           radius))
        return earliest_arrival_time;
    }
    earliest_arrival_time += env.time_resolution;
  }
  return -1.0;
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

void ConstraintTable::interpolatePoint(int agent_id, const Point &from_point, const Point &to_point,
                                       vector<Point> &interpolated_points) const {
  // const double velocity = calculateDistance(from_point, to_point) / env.edge_moving_time;
  const double velocity = env.max_velocities[agent_id];
  const double theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);

  double elapsed_time = 0.0;
  while (elapsed_time < calculateDistance(from_point, to_point) / velocity) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = Point(from_point.x + velocity * cos(theta) * elapsed_time,
                                 from_point.y + velocity * sin(theta) * elapsed_time);
    }
    interpolated_points.emplace_back(interpolated_point);
    elapsed_time += env.check_time_resolution;
  }
  interpolated_points.emplace_back(to_point);

  assert(!interpolated_points.empty());
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point &from_point, const Point &to_point,
                                           double from_time, double to_time, vector<Point> &interpolated_points,
                                           vector<double> &interpolated_times) const {
  assert(from_time < to_time);
  // const double velocity = calculateDistance(from_point, to_point) / env.edge_moving_time;
  const double velocity = env.max_velocities[agent_id];
  const double theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);

  double elapsed_time = 0.0;
  while (elapsed_time < calculateDistance(from_point, to_point) / velocity) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = Point(from_point.x + velocity * cos(theta) * elapsed_time,
                                 from_point.y + velocity * sin(theta) * elapsed_time);
    }
    interpolated_points.emplace_back(interpolated_point);
    interpolated_times.emplace_back(from_time + elapsed_time);
    elapsed_time += env.check_time_resolution;
  }
  interpolated_points.emplace_back(to_point);
  interpolated_times.emplace_back(to_time);

  assert(!interpolated_points.empty());
  assert(interpolated_points.size() == interpolated_times.size());
}

bool ConstraintTable::checkConflicts(const Solution &solution) const {
  for (int agent1_id = 0; agent1_id < solution.size(); ++agent1_id) {
    for (int i = 0; i < solution[agent1_id].size() - 1; ++i) {
      auto [from_point, from_time] = solution[agent1_id][i];
      auto [to_point, to_time] = solution[agent1_id][i + 1];
      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(agent1_id, from_point, to_point, from_time, to_time, interpolated_points,
                           interpolated_times);

      for (auto agent2_id = 0; agent2_id < solution.size(); ++agent2_id) {
        if (agent1_id == agent2_id)
          continue;
        // path conflict
        for (int j = 0; j < solution[agent2_id].size() - 1; ++j) {
          auto [prev_point, prev_time] = solution[agent2_id][j];
          auto [next_point, next_time] = solution[agent2_id][j + 1];

          // check if temporal constraint is satisfied
          if (next_time <= from_time)
            continue;
          if (prev_time >= to_time)
            break;
          // check if spatial constraint is satisfied
          if (calculateDistance(from_point, prev_point) >=
              calculateDistance(from_point, to_point) + env.radii[agent1_id] +
                  calculateDistance(prev_point, next_point) + env.radii[agent2_id] + env.epsilon)
            continue;

          double start_time = max(from_time, prev_time);
          double end_time = min(to_time, next_time);

          auto curr_time = start_time;
          while (curr_time <= end_time) {
            // get point at start_time
            const auto occupied_moving_time = curr_time - prev_time;
            assert(occupied_moving_time >= 0.0);
            const auto occupied_theta = atan2(next_point.y - prev_point.y, next_point.x - prev_point.x);

            auto occupied_point = prev_point;
            if (occupied_theta != 0.0) {
              occupied_point =
                  Point(prev_point.x + env.max_velocities[agent2_id] * cos(occupied_theta) * occupied_moving_time,
                        prev_point.y + env.max_velocities[agent2_id] * sin(occupied_theta) * occupied_moving_time);
            }

            // get point2 at start_time
            const auto moving_time = curr_time - from_time;
            assert(moving_time >= 0.0);
            const auto theta = atan2(to_point.y - from_point.y, to_point.x - from_point.x);

            auto point = from_point;
            if (theta != 0.0) {
              point = Point(from_point.x + env.max_velocities[agent1_id] * cos(theta) * moving_time,
                            from_point.y + env.max_velocities[agent1_id] * sin(theta) * moving_time);
            }

            if (calculateDistance(point, occupied_point) < env.radii[agent1_id] + env.radii[agent2_id]) {
              // cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time " << curr_time
              //      << endl;
              // cout << "From: (" << get<0>(from_point) << ", " << get<1>(from_point) << "), t: " << from_time << endl;
              // cout << "To: (" << get<0>(to_point) << ", " << get<1>(to_point) << "), t: " << to_time << endl;
              // cout << "FTPoint: (" << get<0>(point) << ", " << get<1>(point) << ")" << endl;
              // cout << "Prev: (" << get<0>(prev_point) << ", " << get<1>(prev_point) << "), t: " << prev_time << endl;
              // cout << "Next: (" << get<0>(next_point) << ", " << get<1>(next_point) << "), t: " << next_time << endl;
              // cout << "PNPoint: (" << get<0>(occupied_point) << ", " << get<1>(occupied_point) << ")" << endl;
              // cout << "Distance: " << calculateDistance(point, occupied_point) << endl;
              return true;
            }

            curr_time += env.check_time_resolution;
          }
        }

        // target conflict
        auto [last_point, last_time] = solution[agent2_id].back();
        if (last_time >= to_time)
          continue;
        if (calculateDistance(from_point, last_point) >=
            env.radii[agent1_id] + calculateDistance(from_point, to_point) + env.radii[agent2_id] + env.epsilon)
          continue;
        for (int j = 0; j < interpolated_points.size(); ++j) {
          if (last_time >= interpolated_times[j])
            continue;
          if (calculateDistance(last_point, interpolated_points[j]) < env.radii[agent1_id] + env.radii[agent2_id]) {
            // cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time "
            //      << interpolated_times[j] << endl;
            // cout << "Point: (" << get<0>(interpolated_points[j]) << ", " << get<1>(interpolated_points[j]) << ")"
            //      << endl;
            // cout << "Last Point: (" << get<0>(last_point) << ", " << get<1>(last_point) << ")" << endl;
            return true;
          }
        }
      }
    }
  }
  return false;
}
