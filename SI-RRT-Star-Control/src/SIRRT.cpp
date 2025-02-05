#include "SIRRT.h"

bool SIRRT::run() {
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

    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(random_point, random_velocity, neighbors);
    assert(!neighbors.empty());
    vector<shared_ptr<LLNode>> new_nodes = chooseParent(random_point, random_velocity, safe_interval_table, nodes);
    if (new_nodes.empty()) {
      continue;
    }
    rewire(new_nodes, nodes);

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
    std::tie(path, trajectory) = extractPathAndTrajectory(goal_node);
    return true;
  }

  return false;
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

tuple<Path, Trajectory> SIRRT::extractPathAndTrajectory(const shared_ptr<LLNode> &goal_node) {
  Trajectory temp_trajectory;
  Path temp_path;

  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    // path update
    temp_path.emplace_back(curr_node->point, curr_node->velocity, curr_node->earliest_arrival_time);

    // trajectory update
    validateControl(curr_node->x_control, curr_node->parent->point.x, curr_node->parent->velocity.x, curr_node->point.x,
                    curr_node->velocity.x, env.epsilon);
    validateControl(curr_node->y_control, curr_node->parent->point.y, curr_node->parent->velocity.y, curr_node->point.y,
                    curr_node->velocity.y, env.epsilon);
    XYControl xy_control(curr_node->x_control, curr_node->y_control);
    temp_trajectory.emplace_back(xy_control);

    // recursion
    curr_node = curr_node->parent;
  }
  temp_path.emplace_back(start_point, start_velocity, 0.0);
  temp_trajectory.emplace_back(Control(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Control(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  std::reverse(temp_path.begin(), temp_path.end());
  std::reverse(temp_trajectory.begin(), temp_trajectory.end());

  return {temp_path, temp_trajectory};
}

void SIRRT::getNeighbors(Point point, Velocity velocity, vector<shared_ptr<LLNode>> &neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_cost_to_go = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto &node : nodes) {
    optional<double> cost_to_go = calculateCostToGo(node->point, point, node->velocity, velocity, env.v_max, env.a_max);
    if (cost_to_go == nullopt) {
      continue;
    }
    if (cost_to_go.value() < connection_cost_to_go) {
      // if (constraint_table.obstacleConstrained(agent_id, node->point, point, env.radii[agent_id]))
      //   continue;
      neighbors.emplace_back(node);
    }
  }
}

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point &new_point, const Velocity &new_velocity,
                                               SafeIntervalTable &safe_interval_table,
                                               const vector<shared_ptr<LLNode>> &neighbors) const {
  assert(!nodes.empty());

  auto new_nodes = vector<shared_ptr<LLNode>>();

  for (auto &safe_interval : safe_interval_table.table[new_point]) {
    auto new_node = make_shared<LLNode>(new_point, new_velocity, safe_interval.first, safe_interval.second);
    if (new_node->interval.first >= best_arrival_time)
      continue;

    for (const auto &node : neighbors) {
      if (node->earliest_arrival_time >= new_node->interval.second)
        continue;
      if (node->interval.second <= new_node->interval.first)
        continue;

      // 기존에는 wait 동작을 통해서 충돌을 피했지만, 이제는 충돌을 피하기 위해 가속도를 조절한다.
      auto earliest_arrival_control = constraint_table.getEarliestArrivalControl(
          agent_id, node->point, new_node->point, node->velocity, new_node->velocity, node->earliest_arrival_time,
          new_node->interval.first, new_node->interval.second, env.radii[agent_id]);
      if (earliest_arrival_control == nullopt)
        continue;

      // it doesn't matter whether the control is x or y
      double moving_time = earliest_arrival_control->x_control.control_first.second +
                           earliest_arrival_control->x_control.control_const.second +
                           earliest_arrival_control->x_control.control_last.second;
      double earliest_arrival_time = node->earliest_arrival_time + moving_time;
      if (earliest_arrival_time < new_node->earliest_arrival_time) {
        new_node->earliest_arrival_time = earliest_arrival_time;
        new_node->parent = node;
        new_node->x_control = earliest_arrival_control->x_control;
        new_node->y_control = earliest_arrival_control->y_control;
      }
    }
    if (new_node->parent) {
      new_nodes.push_back(new_node);
    }
  }

  return new_nodes;
}

void SIRRT::rewire(const vector<shared_ptr<LLNode>> &new_nodes, const vector<shared_ptr<LLNode>> &neighbors) {
  assert(!nodes.empty());
  for (auto &new_node : new_nodes) {
    for (auto &node : neighbors) {
      if (new_node->earliest_arrival_time >= node->interval.second)
        continue;
      if (new_node->interval.second <= node->interval.first)
        continue;

      auto earliest_arrival_control = constraint_table.getEarliestArrivalControl(
          agent_id, new_node->point, node->point, new_node->velocity, node->velocity, new_node->earliest_arrival_time,
          node->interval.first, node->interval.second, env.radii[agent_id]);
      if (earliest_arrival_control == nullopt)
        continue;

      double moving_time = earliest_arrival_control->x_control.control_first.second +
                           earliest_arrival_control->x_control.control_const.second +
                           earliest_arrival_control->x_control.control_last.second;
      double earliest_arrival_time = node->earliest_arrival_time + moving_time;
      if (earliest_arrival_time < node->earliest_arrival_time) {
        node->earliest_arrival_time = earliest_arrival_time;
        node->parent = new_node;
        node->x_control = earliest_arrival_control->x_control;
        node->y_control = earliest_arrival_control->y_control;
      }
    }
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  trajectory.clear();
  goal_node = nullptr;
  best_arrival_time = numeric_limits<double>::infinity();
}
