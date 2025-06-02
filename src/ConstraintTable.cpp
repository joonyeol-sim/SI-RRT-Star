#include "ConstraintTable.h"

template<int DIM>
double ConstraintTable<DIM>::TimeToCollision(const State<DIM> &start_state1, double radius1, Velocity<DIM> &v1,
                                        const State<DIM> &start_state2, double radius2, Velocity<DIM> &v2, double start_time,
                                        double end_time) const {
  double combined_radius = radius1 + radius2;
  State<DIM> w = start_state2 - start_state1;
  double c = w.dot(w) - combined_radius * combined_radius;

  if (c < 0) {
    // Agents are already colliding
    return start_time;
  }

  Velocity<DIM> difference_v = v1 - v2;
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

// 선분(에이전트 중심 경로) vs n차원 구(장애물) 충돌 검사
template<int DIM>
bool checkCollisionSegmentCircle(const State<DIM> &A, const State<DIM> &B, double rA, const State<DIM> &C, double rC) {
  double R = rA + rC; // 두 구의 반지름 합

  // n차원 벡터 계산
  Velocity<DIM> AB = toVelocity<DIM>(A, B);
  Velocity<DIM> AC = toVelocity<DIM>(A, C);
  
  double AB_len2 = AB.dot(AB);
  if (AB_len2 < 1e-12) {
    // A와 B가 같은 점인 경우, A와 C의 거리만 확인
    return calculateDistance<DIM>(A, C) <= R;
  }

  // 1) 직선으로 봤을 때 투영 t 계산
  double t = AC.dot(AB) / AB_len2;
  
  // 2) 선분이므로 [0,1] 범위로 clamp
  t = std::max(0.0, std::min(1.0, t));

  // 3) 선분 위 가장 가까운 점 P(t) = A + t * AB
  State<DIM> closest_point = A + toState<DIM>(AB * t);

  // 4) 그 점이 장애물의 중심 C와 R 거리 이하인지 확인
  return calculateDistance<DIM>(closest_point, C) <= R;
}

// Liang-Barsky에 쓸 헬퍼 함수
static bool clipTest(double p, double q, double &u1, double &u2) {
  if (fabs(p) < 1e-12) {
    if (q < 0.0)
      return false;
    return true;
  }

  double r = q / p;
  if (p < 0.0) {
    if (r > u2)
      return false;
    else if (r > u1)
      u1 = r;
  } else {
    if (r < u1)
      return false;
    else if (r < u2)
      u2 = r;
  }
  return true;
}

template<int DIM>
bool checkCollisionSegmentRect(const State<DIM> &A, const State<DIM> &B, double rA, const State<DIM> &rectCenter,
                              const std::array<double, DIM> &dimensions) {
  // n차원 하이퍼직육면체와 선분 충돌 검사 (Liang-Barsky 확장)
  
  // 선분 방향 벡터
  Velocity<DIM> direction = toVelocity<DIM>(A, B);
  
  double t_min = 0.0, t_max = 1.0;

  for (int i = 0; i < DIM; ++i) {
    double half_dim = dimensions[i] / 2.0 + rA;
    double min_bound = rectCenter[i] - half_dim;
    double max_bound = rectCenter[i] + half_dim;
    
    double d = direction[i];
    
    if (std::abs(d) < 1e-12) {
      // 방향 벡터가 이 차원에서 0인 경우
      if (A[i] < min_bound || A[i] > max_bound) {
        return false;
      }
    } else {
      // 교점 계산
      double t1 = (min_bound - A[i]) / d;
      double t2 = (max_bound - A[i]) / d;
      
      if (t1 > t2) std::swap(t1, t2);
      
      t_min = std::max(t_min, t1);
      t_max = std::min(t_max, t2);
      
      if (t_min > t_max) return false;
    }
  }
  
  return (t_min <= 1.0 && t_max >= 0.0);
}

// 2D 호환성을 위한 래퍼 함수
template<int DIM>
bool checkCollisionSegmentRect(const State<DIM> &A, const State<DIM> &B, double rA, const State<DIM> &rectCenter,
                              double rectW, double rectH) {
  static_assert(DIM >= 2, "Need at least 2D for rectangle collision");
  
  std::array<double, DIM> dimensions{};
  dimensions[0] = rectW;
  dimensions[1] = rectH;
  for (int i = 2; i < DIM; ++i) {
    dimensions[i] = 1.0; // 기본값
  }
  
  return checkCollisionSegmentRect<DIM>(A, B, rA, rectCenter, dimensions);
}

template<int DIM>
bool ConstraintTable<DIM>::obstacleConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                          double agent_radius) const {
  for (const auto &obs : env.obstacles) {
    // 원형 장애물인 경우
    if (auto *sphere = dynamic_cast<HyperSphereObstacle<DIM> *>(obs.get())) {
      if (checkCollisionSegmentCircle<DIM>(from_state, to_state, agent_radius, sphere->center, sphere->radius)) {
        return true;
      }
    }
    // 하이퍼직육면체 장애물인 경우 (n차원)
    else if (auto *rect = dynamic_cast<HyperRectangleObstacle<DIM> *>(obs.get())) {
      if (checkCollisionSegmentRect<DIM>(from_state, to_state, agent_radius, rect->center, rect->dimensions)) {
        return true;
      }
    }
  }
  return false;
}

template<int DIM>
bool ConstraintTable<DIM>::targetConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                       double from_time, double to_time, double radius) const {
  for (auto occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    auto [last_state, last_time] = path_table[occupied_agent_id].back();

    if (last_time >= to_time)
      continue;

    if (checkCollisionSegmentCircle(from_state, to_state, radius, last_state, other_radius)) {
      return true;
    }
  }
  return false;
}

template<int DIM>
bool ConstraintTable<DIM>::pathConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                     double from_time, double to_time, double radius) const {
  assert(from_time < to_time);

  // Calculate velocity for agent_id
  State<DIM> direction = to_state - from_state;
  double distance = calculateDistance(from_state, to_state);
  Velocity<DIM> v1;

  if (distance > 1e-9) {
    double speed = env.max_velocities[agent_id];
    for (int i = 0; i < DIM; ++i) {
      v1[i] = speed * direction[i] / distance;
    }
  }

  for (auto occupied_agent_id = 0; occupied_agent_id < static_cast<int>(path_table.size()); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    const auto &other_path = path_table[occupied_agent_id];

    for (int i = 0; i < static_cast<int>(other_path.size()) - 1; ++i) {
      auto [prev_state, prev_time] = other_path[i];
      auto [next_state, next_time] = other_path[i + 1];

      if (next_time <= from_time)
        continue;
      if (prev_time >= to_time)
        break;

      if (calculateDistance(from_state, prev_state) >=
          calculateDistance(from_state, to_state) + radius +
          calculateDistance(prev_state, next_state) + other_radius + env.epsilon)
        continue;

      // Calculate velocity for occupied_agent_id
      State<DIM> other_direction = next_state - prev_state;
      double other_distance = calculateDistance(prev_state, next_state);
      Velocity<DIM> v2;

      if (other_distance > 1e-9) {
        double other_speed = env.max_velocities[occupied_agent_id];
        for (int j = 0; j < DIM; ++j) {
          v2[j] = other_speed * other_direction[j] / other_distance;
        }
      }

      double start_time = std::max(from_time, prev_time);
      double end_time = std::min(to_time, next_time);

      // Calculate start positions at start_time
      State<DIM> start_state1 = from_state;
      State<DIM> start_state2 = prev_state;

      for (int j = 0; j < DIM; ++j) {
        start_state1[j] += v1[j] * (start_time - from_time);
        start_state2[j] += v2[j] * (start_time - prev_time);
      }

      double collision_t = TimeToCollision(start_state1, radius, v1, start_state2, other_radius, v2, start_time, end_time);

      if (collision_t > 0.0) {
        return true;
      }
    }
  }
  return false;
}

template<int DIM>
bool ConstraintTable<DIM>::hardConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                     double from_time, double to_time, double radius) const {
  assert(from_time < to_time);
  const double velocity_from2to = env.max_velocities[agent_id];

  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    for (int i = 0; i < constrained_path.size() - 1; i++) {
      auto [prev_state, prev_time] = constrained_path[i];
      auto [next_state, next_time] = constrained_path[i + 1];

      if (next_time <= from_time)
        continue;
      if (prev_time >= to_time)
        break;

      if (calculateDistance(from_state, prev_state) >=
          calculateDistance(from_state, to_state) + radius +
          calculateDistance(prev_state, next_state) + constrained_radius + env.epsilon)
        continue;

      double start_time = max(from_time, prev_time);
      double end_time = min(to_time, next_time);

      const auto velocity_prev2next = env.max_velocities[agent_id];

      auto curr_time = start_time;
      while (curr_time <= end_time) {
        const auto occupied_moving_time = curr_time - prev_time;
        assert(occupied_moving_time >= 0.0);

        State<DIM> direction = next_state - prev_state;
        double distance = calculateDistance(prev_state, next_state);

        auto occupied_state = prev_state;
        if (distance > 1e-9) {
          for (int j = 0; j < DIM; ++j) {
            occupied_state[j] += velocity_prev2next * (direction[j] / distance) * occupied_moving_time;
          }
        }

        const auto moving_time = curr_time - from_time;
        assert(moving_time >= 0.0);

        State<DIM> my_direction = to_state - from_state;
        double my_distance = calculateDistance(from_state, to_state);

        auto my_state = from_state;
        if (my_distance > 1e-9) {
          for (int j = 0; j < DIM; ++j) {
            my_state[j] += velocity_from2to * (my_direction[j] / my_distance) * moving_time;
          }
        }

        if (calculateDistance(my_state, occupied_state) < radius + constrained_radius + env.epsilon) {
          return true;
        }

        curr_time += env.check_time_resolution;
      }
    }
  }
  return false;
}

template<int DIM>
void ConstraintTable<DIM>::getSafeIntervalTablePath(int agent_id, const State<DIM> &to_state, double radius,
                                               std::vector<Interval> &safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, std::numeric_limits<double>::infinity());

  double eps = env.epsilon;

  for (int occupied_agent_id = 0; occupied_agent_id < (int)path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id)
      continue;
    if (path_table[occupied_agent_id].empty())
      continue;

    double other_radius = env.radii[occupied_agent_id];
    const auto &other_path = path_table[occupied_agent_id];

    for (int i = 0; i < (int)other_path.size() - 1; ++i) {
      auto [prev_state, prev_time] = other_path[i];
      auto [next_state, next_time] = other_path[i + 1];

      if (next_time <= 0.0)
        continue;
      if (prev_time >= std::numeric_limits<double>::infinity())
        break;

      if (prev_time > safe_intervals.back().second)
        break;

      double segment_duration = next_time - prev_time;
      if (segment_duration <= 1e-9)
        continue;

      // Calculate relative position and velocity
      State<DIM> w = prev_state - to_state;

      State<DIM> direction = next_state - prev_state;
      Velocity<DIM> v2;
      for (int j = 0; j < DIM; ++j) {
        v2[j] = direction[j] / segment_duration;
      }

      double combined_radius = radius + other_radius;

      Velocity<DIM> difference_v = v2;
      double a = difference_v.dot(difference_v);
      double b = w.dot(difference_v);
      double c = w.dot(w) - combined_radius * combined_radius;

      double discr = b * b - a * c;
      if (a < 1e-12) {
        if (w.dot(w) <= combined_radius * combined_radius) {
          insertCollisionIntervalToSIT(safe_intervals, prev_time, next_time);
          if (safe_intervals.empty())
            return;
        }
        continue;
      }

      if (discr < 0.0) {
        if (c < 0) {
          insertCollisionIntervalToSIT(safe_intervals, prev_time, next_time);
          if (safe_intervals.empty())
            return;
        }
        continue;
      }

      double sqrt_discr = std::sqrt(discr);
      double tau1 = (-b - sqrt_discr) / a;
      double tau2 = (-b + sqrt_discr) / a;
      if (tau1 > tau2)
        std::swap(tau1, tau2);

      double coll_start = 0.0, coll_end = 0.0;

      if (c < 0.0) {
        coll_start = 0.0;
        coll_end = tau2;
      } else {
        coll_start = tau1;
        coll_end = tau2;
      }

      if (coll_end < 0.0 || coll_start > segment_duration) {
        continue;
      }

      coll_start = std::max(coll_start, 0.0);
      coll_end = std::min(coll_end, segment_duration);
      if (coll_end <= coll_start)
        continue;

      double global_start = prev_time + coll_start;
      double global_end = prev_time + coll_end;

      insertCollisionIntervalToSIT(safe_intervals, global_start, global_end);
      if (safe_intervals.empty())
        return;
    }

    // Target conflict
    auto [final_state, final_time] = other_path.back();
    double dist_sq = calculateDistance(final_state, to_state);
    double combined_radius = radius + other_radius + eps;
    if (dist_sq <= combined_radius) {
      insertCollisionIntervalToSIT(safe_intervals, final_time, std::numeric_limits<double>::infinity());
      if (safe_intervals.empty())
        return;
    }
  }
}

template<int DIM>
void ConstraintTable<DIM>::getSafeIntervalTable(int agent_id, const State<DIM> &to_state, double radius,
                                           vector<Interval> &safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());

  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    bool is_safe = true;
    double collision_start_time = 0.0;

    for (int i = 0; i < constrained_path.size() - 1; ++i) {
      auto [prev_state, prev_time] = constrained_path[i];
      auto [next_state, next_time] = constrained_path[i + 1];

      vector<State<DIM>> interpolated_states;
      vector<double> interpolated_times;
      interpolateStateTime(agent_id, prev_state, next_state, prev_time, next_time,
                          interpolated_states, interpolated_times);

      for (int j = 0; j < interpolated_states.size(); ++j) {
        if (is_safe && calculateDistance(to_state, interpolated_states[j]) <
            radius + constrained_radius + env.epsilon) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (!is_safe && calculateDistance(to_state, interpolated_states[j]) >=
                   radius + constrained_radius + env.epsilon) {
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

template<int DIM>
double ConstraintTable<DIM>::getEarliestArrivalTime(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                               double expand_time, double lower_bound, double upper_bound,
                                               double radius) const {
  double earliest_arrival_time = lower_bound;
  while (earliest_arrival_time < upper_bound) {
    if (env.algorithm == "pp") {
      if (targetConstrained(agent_id, from_state, to_state, earliest_arrival_time - expand_time, earliest_arrival_time,
                            radius))
        return -1.0;
      if (!pathConstrained(agent_id, from_state, to_state, earliest_arrival_time - expand_time, earliest_arrival_time,
                           radius))
        return earliest_arrival_time;
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_state, to_state, earliest_arrival_time - expand_time, earliest_arrival_time,
                           radius))
        return earliest_arrival_time;
    }
    earliest_arrival_time += env.time_resolution;
  }
  return -1.0;
}

template<int DIM>
void ConstraintTable<DIM>::insertCollisionIntervalToSIT(vector<Interval> &safe_intervals, double t_min, double t_max) const {
  assert(t_min >= 0 && t_min < t_max && !safe_intervals.empty());

  int i = 0;
  while (i < safe_intervals.size()) {
    if (t_min >= safe_intervals[i].second) {
      ++i;
      continue;
    }
    if (t_max <= safe_intervals[i].first) {
      break;
    }

    if (t_min <= safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      safe_intervals.erase(safe_intervals.begin() + i);
    } else if (t_min <= safe_intervals[i].first && t_max < safe_intervals[i].second) {
      safe_intervals[i].first = t_max;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      safe_intervals[i].second = t_min;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max < safe_intervals[i].second) {
      safe_intervals.insert(safe_intervals.begin() + i + 1, {t_max, safe_intervals[i].second});
      safe_intervals[i].second = t_min;
      ++i;
    }
  }
}

template<int DIM>
void ConstraintTable<DIM>::interpolateState(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                       vector<State<DIM>> &interpolated_states) const {
  const double velocity = env.max_velocities[agent_id];
  const double distance = calculateDistance(from_state, to_state);

  State<DIM> direction = to_state - from_state;

  double elapsed_time = 0.0;
  while (elapsed_time < distance / velocity) {
    State<DIM> interpolated_state = from_state;
    if (distance > 1e-9) {
      for (int i = 0; i < DIM; ++i) {
        interpolated_state[i] += velocity * (direction[i] / distance) * elapsed_time;
      }
    }
    interpolated_states.emplace_back(interpolated_state);
    elapsed_time += env.check_time_resolution;
  }
  interpolated_states.emplace_back(to_state);

  assert(!interpolated_states.empty());
}

template<int DIM>
void ConstraintTable<DIM>::interpolateStateTime(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                                           double from_time, double to_time, vector<State<DIM>> &interpolated_states,
                                           vector<double> &interpolated_times) const {
  assert(from_time < to_time);
  const double velocity = env.max_velocities[agent_id];
  const double distance = calculateDistance(from_state, to_state);

  State<DIM> direction = to_state - from_state;

  double elapsed_time = 0.0;
  while (elapsed_time < distance / velocity) {
    State<DIM> interpolated_state = from_state;
    if (distance > 1e-9) {
      for (int i = 0; i < DIM; ++i) {
        interpolated_state[i] += velocity * (direction[i] / distance) * elapsed_time;
      }
    }
    interpolated_states.emplace_back(interpolated_state);
    interpolated_times.emplace_back(from_time + elapsed_time);
    elapsed_time += env.check_time_resolution;
  }
  interpolated_states.emplace_back(to_state);
  interpolated_times.emplace_back(to_time);

  assert(!interpolated_states.empty());
  assert(interpolated_states.size() == interpolated_times.size());
}

template<int DIM>
bool ConstraintTable<DIM>::checkConflicts(const Solution<DIM> &solution) const {
  for (int agent1_id = 0; agent1_id < solution.size(); ++agent1_id) {
    for (int i = 0; i < solution[agent1_id].size() - 1; ++i) {
      auto [from_state, from_time] = solution[agent1_id][i];
      auto [to_state, to_time] = solution[agent1_id][i + 1];

      vector<State<DIM>> interpolated_states;
      vector<double> interpolated_times;
      interpolateStateTime(agent1_id, from_state, to_state, from_time, to_time,
                          interpolated_states, interpolated_times);

      for (auto agent2_id = 0; agent2_id < solution.size(); ++agent2_id) {
        if (agent1_id == agent2_id)
          continue;

        // path conflict
        for (int j = 0; j < solution[agent2_id].size() - 1; ++j) {
          auto [prev_state, prev_time] = solution[agent2_id][j];
          auto [next_state, next_time] = solution[agent2_id][j + 1];

          if (next_time <= from_time)
            continue;
          if (prev_time >= to_time)
            break;

          if (calculateDistance(from_state, prev_state) >=
              calculateDistance(from_state, to_state) + env.radii[agent1_id] +
              calculateDistance(prev_state, next_state) + env.radii[agent2_id] + env.epsilon)
            continue;

          double start_time = max(from_time, prev_time);
          double end_time = min(to_time, next_time);

          auto curr_time = start_time;
          while (curr_time <= end_time) {
            const auto occupied_moving_time = curr_time - prev_time;
            assert(occupied_moving_time >= 0.0);

            State<DIM> other_direction = next_state - prev_state;
            double other_distance = calculateDistance(prev_state, next_state);

            auto occupied_state = prev_state;
            if (other_distance > 1e-9) {
              for (int k = 0; k < DIM; ++k) {
                occupied_state[k] += env.max_velocities[agent2_id] *
                                   (other_direction[k] / other_distance) * occupied_moving_time;
              }
            }

            const auto moving_time = curr_time - from_time;
            assert(moving_time >= 0.0);

            State<DIM> my_direction = to_state - from_state;
            double my_distance = calculateDistance(from_state, to_state);

            auto my_state = from_state;
            if (my_distance > 1e-9) {
              for (int k = 0; k < DIM; ++k) {
                my_state[k] += env.max_velocities[agent1_id] *
                             (my_direction[k] / my_distance) * moving_time;
              }
            }

            if (calculateDistance(my_state, occupied_state) < env.radii[agent1_id] + env.radii[agent2_id]) {
              return true;
            }

            curr_time += env.check_time_resolution;
          }
        }

        // target conflict
        auto [last_state, last_time] = solution[agent2_id].back();
        if (last_time >= to_time)
          continue;
        if (calculateDistance(from_state, last_state) >=
            env.radii[agent1_id] + calculateDistance(from_state, to_state) + env.radii[agent2_id] + env.epsilon)
          continue;

        for (int j = 0; j < interpolated_states.size(); ++j) {
          if (last_time >= interpolated_times[j])
            continue;
          if (calculateDistance(last_state, interpolated_states[j]) < env.radii[agent1_id] + env.radii[agent2_id]) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

// 명시적 템플릿 인스턴스화 (필요한 차원들에 대해)
template class ConstraintTable<2>;
template class ConstraintTable<3>;