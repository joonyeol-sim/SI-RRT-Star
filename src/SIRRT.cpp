#include "SIRRT.h"

tuple<Path, Controls> SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id],
                                              safe_interval_table.table[start_point]);
  assert(!safe_interval_table.table[start_point].empty());

  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id],
                                              safe_interval_table.table[goal_point]);
  assert(!safe_interval_table.table[goal_point].empty());

  // initialize start node
  Control start_control = Control(make_tuple(0.0, 0.0), make_tuple(0.0, 0.0), make_tuple(0.0, 0.0));
  auto start_node =
      make_shared<LLNode>(start_point, start_control, 0.0, safe_interval_table.table[start_point].front().second);
  start_node->earliest_arrival_time = 0.0;
  nodes.push_back(start_node);

  // initialize goal node
  Control goal_control = Control(make_tuple(0.0, 0.0), make_tuple(0.0, 0.0), make_tuple(0.0, 0.0));
  goal_node = make_shared<LLNode>(goal_point, goal_control, safe_interval_table.table[goal_point].back().first,
                                       numeric_limits<double>::infinity());
  goal_node->earliest_arrival_time = numeric_limits<double>::infinity();

  int iteration = 0;
  while (true) {
    iteration++;
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    Point new_point = steer(nearest_node, random_point, safe_interval_table);
    if (new_point == make_tuple(-1.0, -1.0)) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_point, neighbors);
    assert(!neighbors.empty());
    vector<shared_ptr<LLNode>> new_nodes = chooseParent(new_point, neighbors, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    rewire(new_nodes, neighbors);

    // check goal
    for (auto& new_node : new_nodes) {
      if (calculateDistance(new_node->point, goal_point) < env.epsilon) {
        if (goal_node->interval.first <= new_node->earliest_arrival_time &&
            new_node->earliest_arrival_time < goal_node->earliest_arrival_time) {
          assert(goal_node->interval.first == new_node->interval.first);
          assert(goal_node->interval.second == new_node->interval.second);
          goal_node = new_node;
          best_arrival_time = goal_node->earliest_arrival_time;
        }
      } else {
        assert(calculateDistance(new_node->point, goal_point) >= env.epsilon);
        nodes.push_back(new_node);
      }
    }

    if (best_arrival_time < numeric_limits<double>::infinity() && iteration >= env.iterations[agent_id]) {
      break;
    }
  }

  if (goal_node->earliest_arrival_time < numeric_limits<double>::infinity()) {
    nodes.push_back(goal_node);
    return updatePathAndControl(goal_node);
  }

  // cout << "No path found!" << endl;
  return make_tuple(path, control_inputs);
}

Point SIRRT::generateRandomPoint() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return goal_point;
  }
  return make_tuple(dis_width(env.gen), dis_height(env.gen));
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::infinity();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

Point SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point,
                   SafeIntervalTable& safe_interval_table) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
  const Point to_point = make_tuple(get<0>(from_node->point) + expand_distance * cos(theta),
                                    get<1>(from_node->point) + expand_distance * sin(theta));

  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) {
    return make_tuple(-1.0, -1.0);
  }

  if (calculateDistance(to_point, goal_point) < env.epsilon) {
    return goal_point;
  }

  if (safe_interval_table.table[to_point].empty()) {
    if (env.algorithm == "pp")
      constraint_table.getSafeIntervalTablePath(agent_id, to_point, env.radii[agent_id],
                                                safe_interval_table.table[to_point]);
    if (safe_interval_table.table[to_point].empty()) {
      return make_tuple(-1.0, -1.0);
    }
  }

  return to_point;
}

tuple<Path, Controls> SIRRT::updatePathAndControl(const shared_ptr<LLNode>& goal_node) const {
  Path path;
  Controls control_inputs;
  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    const auto parent_node = curr_node->parent;
    const auto parent_time = parent_node->earliest_arrival_time;
    const auto curr_time = curr_node->earliest_arrival_time;
    assert(parent_time < curr_time);

    const auto curr_control = curr_node->control;
    const auto& [acceleration, acc_time, dec_time] = curr_control;
    const auto expand_time = std::max(std::get<0>(acc_time) + std::get<0>(dec_time), std::get<1>(acc_time) + std::get<1>(dec_time));
    path.emplace_back(curr_node->point, curr_time);
    control_inputs.emplace_back(curr_control);
    if (parent_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(parent_node->point, curr_time - expand_time);
      control_inputs.emplace_back(make_tuple(0.0, 0.0), make_tuple(0.0, 0.0), make_tuple(0.0, 0.0));
    }
    curr_node = curr_node->parent;
  }
  path.emplace_back(curr_node->point, 0);
  control_inputs.emplace_back(make_tuple(0.0, 0.0), make_tuple(0.0, 0.0), make_tuple(0.0, 0.0));
  reverse(path.begin(), path.end());
  reverse(control_inputs.begin(), control_inputs.end());

  if (calculateDistance(get<0>(path.front()), start_point) >= env.epsilon) {
    throw runtime_error("Start point is not correct!");
  }
  if (calculateDistance(get<0>(path.back()), goal_point) >= env.epsilon) {
    throw runtime_error("Goal point is not correct!");
  }

  return make_tuple(path, control_inputs);
}

void SIRRT::getNeighbors(Point point, vector<shared_ptr<LLNode>>& neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < connection_radius) {
      if (constraint_table.obstacleConstrained(agent_id, node->point, point, env.radii[agent_id])) continue;
      neighbors.emplace_back(node);
    }
  }
}

Control SIRRT::getControlInput(const Point& from_point, const Point& to_point) const {
  auto [from_x, from_y] = from_point;
  auto [to_x, to_y] = to_point;

  // Calculate velocity vector
  Velocity v = std::make_tuple(to_x - from_x, to_y - from_y);

  // Calculate velocity norm
  double v_norm = std::sqrt(std::get<0>(v) * std::get<0>(v) + std::get<1>(v) * std::get<1>(v));

  // Calculate unit vector (v_hat)
  Velocity v_hat;
  if (v_norm > 0) {
    v_hat = std::make_tuple(std::get<0>(v) / v_norm, std::get<1>(v) / v_norm);
  } else {
    v_hat = v;
  }

  // Calculate s (max absolute component of v_hat)
  double s = std::max(std::abs(std::get<0>(v_hat)), std::abs(std::get<1>(v_hat)));

  // Calculate acceleration vector
  Acceleration a;
  if (s > 0) {
    a = std::make_tuple(std::get<0>(v_hat) / s, std::get<1>(v_hat) / s);
  } else {
    a = std::make_tuple(0.0, 0.0);
  }

  // Calculate time
  double t = std::sqrt(s * v_norm);

  // Bang-bang control: accelerate, then decelerate
  AccTime acc_time = std::make_tuple(t, t);
  DecTime dec_time = std::make_tuple(t, t);

  Control control = Control(a, acc_time, dec_time);

  // Verification (similar to Python assertions)
  Point q_1 = std::make_tuple(from_x + 0.5 * std::get<0>(a) * t * t, from_y + 0.5 * std::get<1>(a) * t * t);
  Velocity v_1 = std::make_tuple(std::get<0>(a) * t, std::get<1>(a) * t);
  Point q_f = std::make_tuple(std::get<0>(q_1) + std::get<0>(v_1) * t - 0.5 * std::get<0>(a) * t * t,
                              std::get<1>(q_1) + std::get<1>(v_1) * t - 0.5 * std::get<1>(a) * t * t);
  Velocity v_f = std::make_tuple(std::get<0>(v_1) - std::get<0>(a) * t, std::get<1>(v_1) - std::get<1>(a) * t);

  assert(std::abs(std::get<0>(q_f) - to_x) < env.epsilon && std::abs(std::get<1>(q_f) - to_y) < env.epsilon);
  assert(std::abs(std::get<0>(v_f)) < env.epsilon && std::abs(std::get<1>(v_f)) < env.epsilon);

  return control;
}

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
                                               SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  auto new_nodes = vector<shared_ptr<LLNode>>();

  for (auto& safe_interval : safe_interval_table.table[new_point]) {
    auto control = Control(make_tuple(-1.0, -1.0), make_tuple(-1.0, -1.0), make_tuple(-1.0, -1.0));
    auto new_node = make_shared<LLNode>(new_point, control, safe_interval.first, safe_interval.second);
    if (new_node->interval.first >= best_arrival_time) continue;
    for (const auto& neighbor : neighbors) {
      if (neighbor->earliest_arrival_time >= new_node->earliest_arrival_time) continue;

      // calculate t1 and t2
      control = getControlInput(neighbor->point, new_point);
      const auto& [acceleration, acc_time, dec_time] = control;
      const double expand_time =
          std::max(std::get<0>(acc_time) + std::get<0>(dec_time), std::get<1>(acc_time) + std::get<1>(dec_time));
      const double lower_bound = neighbor->earliest_arrival_time + expand_time;
      const double upper_bound = neighbor->interval.second + expand_time;

      if (lower_bound >= best_arrival_time) continue;
      if (lower_bound >= new_node->interval.second) continue;
      if (upper_bound <= new_node->interval.first) continue;

      // calculate Trajectory
      Path trajectory = {};
      constraint_table.generateTrajectory(neighbor->point, new_point, control, trajectory);

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, trajectory, expand_time, max(new_node->interval.first, lower_bound),
          min(new_node->interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) continue;

      if (earliest_arrival_time < new_node->earliest_arrival_time) {
        new_node->control = control;
        new_node->earliest_arrival_time = earliest_arrival_time;
        new_node->parent = neighbor;
      }
    }
    if (new_node->parent) {
      new_nodes.push_back(new_node);
    }
  }

  return new_nodes;
}

void SIRRT::rewire(const vector<shared_ptr<LLNode>>& new_nodes, const vector<shared_ptr<LLNode>>& neighbors) {
  assert(!neighbors.empty());
  for (auto& new_node : new_nodes) {
    for (auto& neighbor : neighbors) {
      if (neighbor->interval.first >= best_arrival_time) continue;
      if (new_node->earliest_arrival_time >= neighbor->earliest_arrival_time) continue;

      // calculate t1 and t2
      auto control = getControlInput(new_node->point, neighbor->point);
      const auto& [acceleration, acc_time, dec_time] = control;
      const double expand_time =
          std::max(std::get<0>(acc_time) + std::get<0>(dec_time), std::get<1>(acc_time) + std::get<1>(dec_time));

      const double lower_bound = new_node->earliest_arrival_time + expand_time;
      const double upper_bound = new_node->interval.second + expand_time;

      if (lower_bound >= best_arrival_time) continue;
      if (lower_bound >= neighbor->interval.second) continue;
      if (upper_bound <= neighbor->interval.first) continue;

      // calculate Trajectory
      Path trajectory = {};
      constraint_table.generateTrajectory(new_node->point, neighbor->point, control, trajectory);

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, trajectory, expand_time, max(neighbor->interval.first, lower_bound),
          min(neighbor->interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) continue;

      if (earliest_arrival_time < neighbor->earliest_arrival_time) {
        neighbor->control = control;
        neighbor->earliest_arrival_time = earliest_arrival_time;
        neighbor->parent = new_node;
      }
    }
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
  best_arrival_time = numeric_limits<double>::infinity();
}
