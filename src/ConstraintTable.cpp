#include "ConstraintTable.h"

bool ConstraintTable::obstacleConstrained(int agent_id, const Point& from_point, const Point& to_point,
                                          double radius) const {
  vector<Point> interpolated_points;
  interpolatePoint(agent_id, from_point, to_point, interpolated_points);
  for (auto& interpolated_point : interpolated_points) {
    for (const auto& obstacle : env.obstacles) {
      if (obstacle->constrained(interpolated_point, radius)) return true;
    }
  }
  return false;
}

bool ConstraintTable::targetConstrained(int agent_id, const Path& trajecotry, double earliest_start_time, double radius) const {
  const Point from_point = get<0>(trajecotry[0]);
  const Point to_point = get<0>(trajecotry.back());
  const double from_time = get<1>(trajecotry[0]) + earliest_start_time;
  const double to_time = get<1>(trajecotry.back()) + earliest_start_time;

  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // target conflict
    auto [last_point, last_time] = path_table[occupied_agent_id].back();
    // check if temporal constraint is satisfied
    if (last_time >= to_time) continue;
    // check if spatial constraint is satisfied
    if (calculateDistance(from_point, last_point) >=
        radius + calculateDistance(from_point, to_point) + env.radii[occupied_agent_id] + env.epsilon)
      continue;

    for (const auto& state : trajecotry) {
      auto this_point = get<0>(state);
      auto this_time = get<1>(state) + earliest_start_time;

      if (last_time >= this_time) continue;
      if (calculateDistance(last_point, this_point) < radius + env.radii[occupied_agent_id] + env.epsilon) {
        return true;
      }
    }
  }
  return false;
}

bool ConstraintTable::pathConstrained(int agent_id, const Path& trajecotry, double earliest_start_time, double radius) const {
  const Point from_point = get<0>(trajecotry[0]);
  const Point to_point = get<0>(trajecotry.back());
  const double from_time = get<1>(trajecotry[0]) + earliest_start_time;
  const double to_time = get<1>(trajecotry.back()) + earliest_start_time;

  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];
      Control control = control_inputs_table[occupied_agent_id][i + 1];
      const auto& [acceleration, acc_time, dec_time] = control;

      // check if temporal constraint is satisfied
      if (next_time <= from_time) continue;
      if (prev_time >= to_time) break;
      // check if spatial constraint is satisfied
      if (calculateDistance(from_point, prev_point) >= calculateDistance(from_point, to_point) + radius +
                                                           calculateDistance(prev_point, next_point) +
                                                           env.radii[occupied_agent_id] + env.epsilon)
        continue;

      for (const auto& state : trajecotry) {
        auto [curr_point, curr_time] = state;
        // find the point at curr_time using the control inputs
        Point occupied_point = calculate_state(prev_point, acc_time, acceleration, curr_time + earliest_start_time - prev_time);

        if (calculateDistance(curr_point, occupied_point) < radius + env.radii[occupied_agent_id] + env.epsilon) {
          return true;
        }
      }
    }
  }
  return false;
}

// bool ConstraintTable::hardConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
//                                       double to_time, double radius) const {
//   assert(from_time < to_time);
//   // const double velocity_from2to = calculateDistance(from_point, to_point) / env.edge_moving_time;
//   const double velocity_from2to = env.max_velocities[agent_id];
//   for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
//     for (int i = 0; i < constrained_path.size() - 1; i++) {
//       auto [prev_point, prev_time] = constrained_path[i];
//       auto [next_point, next_time] = constrained_path[i + 1];
//
//       // check if temporal constraint is satisfied
//       if (next_time <= from_time) continue;
//       if (prev_time >= to_time) break;
//       // check if spatial constraint is satisfied
//       if (calculateDistance(from_point, prev_point) >= calculateDistance(from_point, to_point) + radius +
//                                                            calculateDistance(prev_point, next_point) +
//                                                            constrained_radius + env.epsilon)
//         continue;
//
//       double start_time = max(from_time, prev_time);
//       double end_time = min(to_time, next_time);
//
//       // const auto velocity_prev2next = calculateDistance(prev_point, next_point) / env.edge_moving_time;
//       const auto velocity_prev2next = env.max_velocities[agent_id];
//
//       auto curr_time = start_time;
//       while (curr_time <= end_time) {
//         // get point at start_time
//         const auto occupied_moving_time = curr_time - prev_time;
//         assert(occupied_moving_time >= 0.0);
//         const auto occupied_theta =
//             atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
//
//         auto occupied_point = prev_point;
//         if (occupied_theta != 0.0) {
//           occupied_point =
//               make_tuple(get<0>(prev_point) + velocity_prev2next * cos(occupied_theta) * occupied_moving_time,
//                          get<1>(prev_point) + velocity_prev2next * sin(occupied_theta) * occupied_moving_time);
//         }
//
//         // get point2 at start_time
//         const auto moving_time = curr_time - from_time;
//         assert(moving_time >= 0.0);
//         const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
//
//         auto point = from_point;
//         if (theta != 0.0) {
//           point = make_tuple(get<0>(from_point) + velocity_from2to * cos(theta) * moving_time,
//                              get<1>(from_point) + velocity_from2to * sin(theta) * moving_time);
//         }
//
//         if (calculateDistance(point, occupied_point) < radius + constrained_radius + env.epsilon) {
//           return true;
//         }
//
//         curr_time += env.check_time_resolution;
//       }
//     }
//   }
//   return false;
// }

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point& to_point, double radius,
                                               vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    bool is_safe = true;
    double collision_start_time = 0.0;
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];
      auto control = control_inputs_table[occupied_agent_id][i + 1];

      Path trajectory = {};
      generateTrajectory(prev_point, next_point, control, trajectory);
      for (const auto& state : trajectory) {
        const Point occupied_point = get<0>(state);
        const double occupied_time = get<1>(state) + prev_time;

        if (is_safe &&
            calculateDistance(to_point, occupied_point) < radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = false;
          collision_start_time = occupied_time;
        } else if (!is_safe && calculateDistance(to_point, occupied_point) >=
                                   radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = true;
          assert(collision_start_time < occupied_time);
          insertCollisionIntervalToSIT(safe_intervals, collision_start_time, occupied_time);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {  // target conflict
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time, numeric_limits<double>::infinity());
      if (safe_intervals.empty()) return;
    }
  }
}

// void ConstraintTable::getSafeIntervalTable(int agent_id, const Point& to_point, double radius,
//                                            vector<Interval>& safe_intervals) const {
//   assert(safe_intervals.empty());
//   safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
//   for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
//     bool is_safe = true;
//     double collision_start_time = 0.0;
//     for (int i = 0; i < constrained_path.size() - 1; ++i) {
//       auto [prev_point, prev_time] = constrained_path[i];
//       auto [next_point, next_time] = constrained_path[i + 1];
//
//       vector<Point> interpolated_points;
//       vector<double> interpolated_times;
//       interpolatePointTime(agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
//                            interpolated_times);
//       for (int j = 0; j < interpolated_points.size(); ++j) {
//         if (is_safe &&
//             calculateDistance(to_point, interpolated_points[j]) < radius + constrained_radius + env.epsilon) {
//           is_safe = false;
//           collision_start_time = interpolated_times[j];
//         } else if (!is_safe &&
//                    calculateDistance(to_point, interpolated_points[j]) >= radius + constrained_radius + env.epsilon)
//                    {
//           is_safe = true;
//           assert(collision_start_time < interpolated_times[j]);
//           insertCollisionIntervalToSIT(safe_intervals, collision_start_time, interpolated_times[j]);
//           if (safe_intervals.empty()) return;
//         }
//       }
//     }
//
//     if (!is_safe) {
//       insertCollisionIntervalToSIT(safe_intervals, collision_start_time, get<1>(constrained_path.back()));
//       if (safe_intervals.empty()) return;
//     }
//   }
// }

double ConstraintTable::getEarliestArrivalTime(int agent_id, const Path& trajectory, double expand_time,
                                               double lower_bound, double upper_bound, double radius) const {
  double earliest_arrival_time = lower_bound;
  while (earliest_arrival_time < upper_bound) {
    if (env.algorithm == "pp") {
      double earliest_start_time = earliest_arrival_time - expand_time;
      if (targetConstrained(agent_id, trajectory, earliest_start_time, radius))
        return -1.0;
      if (!pathConstrained(agent_id, trajectory, earliest_start_time, radius))
        return earliest_arrival_time;
    }
    earliest_arrival_time += env.time_resolution;
  }
  return -1.0;
}

void ConstraintTable::insertCollisionIntervalToSIT(vector<Interval>& safe_intervals, double t_min, double t_max) const {
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

void ConstraintTable::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                                       vector<Point>& interpolated_points) const {
  // const double velocity = calculateDistance(from_point, to_point) / env.edge_moving_time;
  const double velocity = env.max_velocities[agent_id];
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

  double elapsed_time = 0.0;
  while (elapsed_time < calculateDistance(from_point, to_point) / velocity) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = make_tuple(get<0>(from_point) + velocity * cos(theta) * elapsed_time,
                                      get<1>(from_point) + velocity * sin(theta) * elapsed_time);
    }
    interpolated_points.emplace_back(interpolated_point);
    elapsed_time += env.check_time_resolution;
  }
  interpolated_points.emplace_back(to_point);

  assert(!interpolated_points.empty());
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, vector<Point>& interpolated_points,
                                           vector<double>& interpolated_times) const {
  assert(from_time < to_time);
  // const double velocity = calculateDistance(from_point, to_point) / env.edge_moving_time;
  const double velocity = env.max_velocities[agent_id];
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

  double elapsed_time = 0.0;
  while (elapsed_time < calculateDistance(from_point, to_point) / velocity) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = make_tuple(get<0>(from_point) + velocity * cos(theta) * elapsed_time,
                                      get<1>(from_point) + velocity * sin(theta) * elapsed_time);
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

Point ConstraintTable::calculate_state(const Point& initial_state, const std::tuple<double, double>& acc_time,
                                       const std::tuple<double, double>& acceleration, double elapsed_time) {
  auto [x0, y0] = initial_state;
  auto [t1x, t1y] = acc_time;
  auto [ax, ay] = acceleration;

  auto calc_coord = [](double t, double p0, double t1, double acc) {
    if (t <= t1) {
      return p0 + 0.5 * acc * t * t;
    } else {
      double x1 = p0 + 0.5 * acc * t1 * t1;
      double v1 = acc * t1;
      double dt = t - t1;
      return x1 + v1 * dt - 0.5 * acc * dt * dt;
    }
  };

  double x = calc_coord(elapsed_time, x0, t1x, ax);
  double y = calc_coord(elapsed_time, y0, t1y, ay);

  return std::make_tuple(x, y);
}

void ConstraintTable::generateTrajectory(const Point& from_point, const Point& to_point, const Control& control,
                                         Path& trajectory) const {
  auto [acceleration, acc_time, dec_time] = control;
  auto [t1_x, t1_y] = acc_time;
  auto [t2_x, t2_y] = dec_time;

  double total_time = std::max(t1_x + t2_x, t1_y + t2_y);

  for (double elapsed_time = 0.0; elapsed_time < total_time; elapsed_time += env.check_time_resolution) {
    Point new_point = calculate_state(from_point, acc_time, acceleration, elapsed_time);
    trajectory.emplace_back(new_point, elapsed_time);
  }

  trajectory.emplace_back(to_point, total_time);
}

bool ConstraintTable::checkConflicts(const Solution& solution) const {
  for (int agent1_id = 0; agent1_id < solution.size(); ++agent1_id) {
    for (int i = 0; i < solution[agent1_id].size() - 1; ++i) {
      auto [from_point, from_time] = solution[agent1_id][i];
      auto [to_point, to_time] = solution[agent1_id][i + 1];
      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(agent1_id, from_point, to_point, from_time, to_time, interpolated_points,
                           interpolated_times);

      for (auto agent2_id = 0; agent2_id < solution.size(); ++agent2_id) {
        if (agent1_id == agent2_id) continue;
        // path conflict
        for (int j = 0; j < solution[agent2_id].size() - 1; ++j) {
          auto [prev_point, prev_time] = solution[agent2_id][j];
          auto [next_point, next_time] = solution[agent2_id][j + 1];

          // check if temporal constraint is satisfied
          if (next_time <= from_time) continue;
          if (prev_time >= to_time) break;
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
            const auto occupied_theta =
                atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));

            auto occupied_point = prev_point;
            if (occupied_theta != 0.0) {
              occupied_point = make_tuple(
                  get<0>(prev_point) + env.max_velocities[agent2_id] * cos(occupied_theta) * occupied_moving_time,
                  get<1>(prev_point) + env.max_velocities[agent2_id] * sin(occupied_theta) * occupied_moving_time);
            }

            // get point2 at start_time
            const auto moving_time = curr_time - from_time;
            assert(moving_time >= 0.0);
            const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

            auto point = from_point;
            if (theta != 0.0) {
              point = make_tuple(get<0>(from_point) + env.max_velocities[agent1_id] * cos(theta) * moving_time,
                                 get<1>(from_point) + env.max_velocities[agent1_id] * sin(theta) * moving_time);
            }

            if (calculateDistance(point, occupied_point) < env.radii[agent1_id] + env.radii[agent2_id]) {
              cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time " << curr_time
                   << endl;
              cout << "From: (" << get<0>(from_point) << ", " << get<1>(from_point) << "), t: " << from_time << endl;
              cout << "To: (" << get<0>(to_point) << ", " << get<1>(to_point) << "), t: " << to_time << endl;
              cout << "FTPoint: (" << get<0>(point) << ", " << get<1>(point) << ")" << endl;
              cout << "Prev: (" << get<0>(prev_point) << ", " << get<1>(prev_point) << "), t: " << prev_time << endl;
              cout << "Next: (" << get<0>(next_point) << ", " << get<1>(next_point) << "), t: " << next_time << endl;
              cout << "PNPoint: (" << get<0>(occupied_point) << ", " << get<1>(occupied_point) << ")" << endl;
              cout << "Distance: " << calculateDistance(point, occupied_point) << endl;
              return true;
            }

            curr_time += env.check_time_resolution;
          }
        }

        // target conflict
        auto [last_point, last_time] = solution[agent2_id].back();
        if (last_time >= to_time) continue;
        if (calculateDistance(from_point, last_point) >=
            env.radii[agent1_id] + calculateDistance(from_point, to_point) + env.radii[agent2_id] + env.epsilon)
          continue;
        for (int j = 0; j < interpolated_points.size(); ++j) {
          if (last_time >= interpolated_times[j]) continue;
          if (calculateDistance(last_point, interpolated_points[j]) < env.radii[agent1_id] + env.radii[agent2_id]) {
            cout << "Agent " << agent1_id << " and Agent " << agent2_id << " have a conflict at time "
                 << interpolated_times[j] << endl;
            cout << "Point: (" << get<0>(interpolated_points[j]) << ", " << get<1>(interpolated_points[j]) << ")"
                 << endl;
            cout << "Last Point: (" << get<0>(last_point) << ", " << get<1>(last_point) << ")" << endl;
            return true;
          }
        }
      }
    }
  }
  return false;
}