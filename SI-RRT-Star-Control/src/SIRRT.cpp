#include "SIRRT.h"

ControlPath SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id],
                                              safe_interval_table.table[start_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, start_point, env.radii[agent_id],
                                          safe_interval_table.table[start_point]);
  assert(!safe_interval_table.table[start_point].empty());

  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id],
                                              safe_interval_table.table[goal_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, goal_point, env.radii[agent_id],
                                          safe_interval_table.table[goal_point]);
  assert(!safe_interval_table.table[goal_point].empty());

  // initialize start node
  auto start_node =
      make_shared<LLNode>(start_point, start_velocity, 0.0, safe_interval_table.table[start_point].front().second);
  start_node->earliest_arrival_time = 0.0;
  nodes.push_back(start_node);

  // initialize goal node
  auto goal_node = make_shared<LLNode>(goal_point, goal_velocity, safe_interval_table.table[goal_point].back().first,
                                       numeric_limits<double>::infinity());
  goal_node->earliest_arrival_time = numeric_limits<double>::infinity();

  int iteration = 0;
  while (true) {
    iteration++;
    if (best_arrival_time < numeric_limits<double>::infinity() && iteration >= env.iterations[agent_id]) {
      break;
    }
    Point random_point;
    Velocity random_velocity;
    std::tie(random_point, random_velocity) = generateRandomState();
    // get safe interval table
    if (safe_interval_table.table[random_point].empty()) {
      if (env.algorithm == "pp")
        constraint_table.getSafeIntervalTablePath(agent_id, random_point, env.radii[agent_id],
                                                  safe_interval_table.table[random_point]);
      else if (env.algorithm == "cbs")
        constraint_table.getSafeIntervalTable(agent_id, random_point, env.radii[agent_id],
                                              safe_interval_table.table[random_point]);
      if (safe_interval_table.table[random_point].empty()) {
        continue;
      }
    }

    vector<shared_ptr<LLNode>> new_nodes = chooseParent(random_point, random_velocity, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    // rewire(new_nodes, neighbors);

    // check goal
    for (auto &new_node : new_nodes) {
      if (new_node->point == goal_point && new_node->velocity == goal_velocity) {
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
  }

  if (goal_node->earliest_arrival_time < numeric_limits<double>::infinity()) {
    nodes.push_back(goal_node);
    control_path = updatePath(goal_node);
    return control_path;
  }

  // cout << "No path found!" << endl;
  return control_path;
}

pair<Point, Velocity> SIRRT::generateRandomState() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return {goal_point, goal_velocity};
  }
  return {Point(dis_width(env.gen), dis_height(env.gen)), Velocity(dis_velocity(env.gen), dis_velocity(env.gen))};
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point &point) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::infinity();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto &node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point &point, const Velocity &velocity) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_cost_to_go = numeric_limits<double>::infinity();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto &node : nodes) {
    optional<double> cost_to_go = calculateCostToGo(node->point, point, node->velocity, velocity, env.v_max, env.a_max);
    // When the cost_to_go is negative, it means that the node is not reachable.
    if (cost_to_go == std::nullopt) {
      continue;
    }

    if (cost_to_go.value() < min_cost_to_go) {
      min_cost_to_go = cost_to_go.value();
      nearest_node = node;
    }
  }

  return nearest_node;
}

ControlPath SIRRT::updatePath(const shared_ptr<LLNode> &goal_node) const {
  std::vector<Control> x_controls;
  std::vector<Control> y_controls;

  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    validateControl(curr_node->x_control, curr_node->parent->point.x, curr_node->parent->velocity.x, curr_node->point.x,
                    curr_node->velocity.x, env.epsilon);
    validateControl(curr_node->y_control, curr_node->parent->point.y, curr_node->parent->velocity.y, curr_node->point.y,
                    curr_node->velocity.y, env.epsilon);
    x_controls.push_back(curr_node->x_control);
    y_controls.push_back(curr_node->y_control);
    curr_node = curr_node->parent;
  }

  std::reverse(x_controls.begin(), x_controls.end());
  std::reverse(y_controls.begin(), y_controls.end());

  return {std::move(x_controls), std::move(y_controls)};
}

void SIRRT::getNeighbors(Point point, vector<shared_ptr<LLNode>> &neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto &node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < connection_radius) {
      if (constraint_table.obstacleConstrained(agent_id, node->point, point, env.radii[agent_id]))
        continue;
      neighbors.emplace_back(node);
    }
  }
}

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point &new_point, const Velocity &new_velocity,
                                               SafeIntervalTable &safe_interval_table) const {
  auto new_nodes = vector<shared_ptr<LLNode>>();

  for (auto &safe_interval : safe_interval_table.table[new_point]) {
    auto new_node = make_shared<LLNode>(new_point, new_velocity, safe_interval.first, safe_interval.second);
    if (new_node->interval.first >= best_arrival_time)
      continue;

    for (const auto &node : nodes) {
      if (node->earliest_arrival_time >= new_node->earliest_arrival_time)
        continue;

      auto cost_with_controls = calculateCostToGoWithControls(node->point, new_node->point, node->velocity,
                                                              new_node->velocity, env.v_max, env.a_max);
      if (!cost_with_controls)
        continue;

      auto [expand_time, x_control, y_control] = std::move(*cost_with_controls);
      double earliest_arrival_time = node->earliest_arrival_time + expand_time;
      if (earliest_arrival_time < new_node->earliest_arrival_time) {
        // 검증: 부모 상태(node)에서 새 노드(new_node)로의 전이가 올바른지 확인
        double tol = env.epsilon; // 허용 오차
        validateControl(*x_control, node->point.x, node->velocity.x, new_node->point.x, new_node->velocity.x, tol);
        validateControl(*y_control, node->point.y, node->velocity.y, new_node->point.y, new_node->velocity.y, tol);

        new_node->earliest_arrival_time = earliest_arrival_time;
        new_node->parent = node;
        new_node->x_control = *x_control;
        new_node->y_control = *y_control;
      }
    }
    if (new_node->parent) {
      new_nodes.push_back(new_node);
    }
  }

  return new_nodes;
}

// void SIRRT::rewire(const vector<shared_ptr<LLNode>> &new_nodes, const vector<shared_ptr<LLNode>> &neighbors) {
//   assert(!neighbors.empty());
//   for (auto &new_node : new_nodes) {
//     for (auto &neighbor : neighbors) {
//       if (neighbor->interval.first >= best_arrival_time)
//         continue;
//       if (new_node->earliest_arrival_time >= neighbor->earliest_arrival_time)
//         continue;
//       const double expand_time = calculateDistance(neighbor->point, new_node->point) / env.max_velocities[agent_id];
//       const double lower_bound = new_node->earliest_arrival_time + expand_time;
//       const double upper_bound = new_node->interval.second + expand_time;
//
//       if (lower_bound >= best_arrival_time)
//         continue;
//       if (lower_bound >= neighbor->interval.second)
//         continue;
//       if (upper_bound <= neighbor->interval.first)
//         continue;
//
//       const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
//           agent_id, new_node->point, neighbor->point, expand_time, max(neighbor->interval.first, lower_bound),
//           min(neighbor->interval.second, upper_bound), env.radii[agent_id]);
//       if (earliest_arrival_time < 0.0)
//         continue;
//
//       if (earliest_arrival_time < neighbor->earliest_arrival_time) {
//         neighbor->earliest_arrival_time = earliest_arrival_time;
//         neighbor->parent = new_node;
//       }
//     }
//   }
// }

void SIRRT::release() {
  nodes.clear();
  goal_node = nullptr;
  best_arrival_time = numeric_limits<double>::infinity();
}
