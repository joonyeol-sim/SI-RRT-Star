#include "SIRRT.h"

Path SIRRT::run() {
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
  auto start_node = make_shared<LLNode>(start_point, 0.0, safe_interval_table.table[start_point].front().second);
  start_node->earliest_arrival_time = 0.0;
  nodes.push_back(start_node);

  // initialize goal node
  auto goal_node = make_shared<LLNode>(goal_point, safe_interval_table.table[goal_point].back().first,
                                       numeric_limits<double>::infinity());
  goal_node->earliest_arrival_time = numeric_limits<double>::infinity();

  int iteration = 0;
  while (true) {
    iteration++;
    if (best_arrival_time < numeric_limits<double>::infinity() && iteration >= env.iterations[agent_id]) {
      break;
    }
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    Point new_point = steer(nearest_node, random_point, safe_interval_table);
    if (new_point == Point(-1.0, -1.0)) {
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
    for (auto &new_node : new_nodes) {
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
  }

  if (goal_node->earliest_arrival_time < numeric_limits<double>::infinity()) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node);
    return path;
  }

  // cout << "No path found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return goal_point;
  }
  return Point(dis_width(env.gen), dis_height(env.gen));
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

Point SIRRT::steer(const shared_ptr<LLNode> &from_node, const Point &random_point,
                   SafeIntervalTable &safe_interval_table) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta = atan2(random_point.y - from_node->point.y, random_point.x - from_node->point.x);
  const Point to_point =
      Point(from_node->point.x + expand_distance * cos(theta), from_node->point.y + expand_distance * sin(theta));

  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) {
    return Point(-1.0, -1.0);
  }

  if (calculateDistance(to_point, goal_point) < env.epsilon) {
    return goal_point;
  }

  if (safe_interval_table.table[to_point].empty()) {
    if (env.algorithm == "pp")
      constraint_table.getSafeIntervalTablePath(agent_id, to_point, env.radii[agent_id],
                                                safe_interval_table.table[to_point]);
    else if (env.algorithm == "cbs")
      constraint_table.getSafeIntervalTable(agent_id, to_point, env.radii[agent_id],
                                            safe_interval_table.table[to_point]);
    if (safe_interval_table.table[to_point].empty()) {
      return Point(-1.0, -1.0);
    }
  }

  return to_point;
}

Path SIRRT::updatePath(const shared_ptr<LLNode> &goal_node) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    const auto prev_node = curr_node->parent;
    const auto prev_time = prev_node->earliest_arrival_time;
    const auto curr_time = curr_node->earliest_arrival_time;
    assert(prev_time < curr_time);

    const auto expand_time = calculateDistance(prev_node->point, curr_node->point) / env.max_velocities[agent_id];
    path.emplace_back(curr_node->point, curr_time);
    if (prev_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(prev_node->point, curr_time - expand_time);
    }
    curr_node = curr_node->parent;
  }
  path.emplace_back(curr_node->point, 0);
  reverse(path.begin(), path.end());

  if (calculateDistance(get<0>(path.front()), start_point) >= env.epsilon) {
    throw runtime_error("Start point is not correct!");
  }
  if (calculateDistance(get<0>(path.back()), goal_point) >= env.epsilon) {
    throw runtime_error("Goal point is not correct!");
  }

  return path;
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

vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point &new_point, const vector<shared_ptr<LLNode>> &neighbors,
                                               SafeIntervalTable &safe_interval_table) const {
  assert(!neighbors.empty());

  auto new_nodes = vector<shared_ptr<LLNode>>();

  for (auto &safe_interval : safe_interval_table.table[new_point]) {
    auto new_node = make_shared<LLNode>(new_point, safe_interval.first, safe_interval.second);
    if (new_node->interval.first >= best_arrival_time)
      continue;

    for (const auto &neighbor : neighbors) {
      if (neighbor->earliest_arrival_time >= new_node->earliest_arrival_time)
        continue;
      const double expand_time = calculateDistance(new_node->point, neighbor->point) / env.max_velocities[agent_id];
      const double lower_bound = neighbor->earliest_arrival_time + expand_time;
      const double upper_bound = neighbor->interval.second + expand_time;

      if (lower_bound >= best_arrival_time)
        continue;
      if (lower_bound >= new_node->interval.second)
        continue;
      if (upper_bound <= new_node->interval.first)
        continue;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->point, new_node->point, expand_time, max(new_node->interval.first, lower_bound),
          min(new_node->interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0)
        continue;

      if (earliest_arrival_time < new_node->earliest_arrival_time) {
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

void SIRRT::rewire(const vector<shared_ptr<LLNode>> &new_nodes, const vector<shared_ptr<LLNode>> &neighbors) {
  assert(!neighbors.empty());
  for (auto &new_node : new_nodes) {
    for (auto &neighbor : neighbors) {
      if (neighbor->interval.first >= best_arrival_time)
        continue;
      if (new_node->earliest_arrival_time >= neighbor->earliest_arrival_time)
        continue;
      const double expand_time = calculateDistance(neighbor->point, new_node->point) / env.max_velocities[agent_id];
      const double lower_bound = new_node->earliest_arrival_time + expand_time;
      const double upper_bound = new_node->interval.second + expand_time;

      if (lower_bound >= best_arrival_time)
        continue;
      if (lower_bound >= neighbor->interval.second)
        continue;
      if (upper_bound <= neighbor->interval.first)
        continue;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, new_node->point, neighbor->point, expand_time, max(neighbor->interval.first, lower_bound),
          min(neighbor->interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0)
        continue;

      if (earliest_arrival_time < neighbor->earliest_arrival_time) {
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
