#include "SIRRT.h"

template<int DIM>
Path<DIM> SIRRT<DIM>::run() {
  release();
  SafeIntervalTable<DIM> safe_interval_table(env);

  // initialize start and goal safe intervals
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_state, env.radii[agent_id],
                                              safe_interval_table.table[start_state]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, start_state, env.radii[agent_id],
                                          safe_interval_table.table[start_state]);
  assert(!safe_interval_table.table[start_state].empty());

  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_state, env.radii[agent_id],
                                              safe_interval_table.table[goal_state]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, goal_state, env.radii[agent_id],
                                          safe_interval_table.table[goal_state]);
  assert(!safe_interval_table.table[goal_state].empty());

  // initialize start node
  auto start_node = make_shared<LLNode<DIM>>(start_state, 0.0, safe_interval_table.table[start_state].front().second);
  start_node->earliest_arrival_time = 0.0;
  nodes.push_back(start_node);

  // initialize goal node
  auto goal_node = make_shared<LLNode<DIM>>(goal_state, safe_interval_table.table[goal_state].back().first,
                                            numeric_limits<double>::infinity());
  goal_node->earliest_arrival_time = numeric_limits<double>::infinity();

  int iteration = 0;
  while (true) {
    iteration++;
    if (best_arrival_time < numeric_limits<double>::infinity() && iteration >= env.iterations[agent_id]) {
      break;
    }

    State<DIM> random_state = generateRandomState();
    const shared_ptr<LLNode<DIM>> nearest_node = getNearestNode(random_state);
    State<DIM> new_state = steer(nearest_node, random_state, safe_interval_table);
    if (!isValidState(new_state)) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode<DIM>>> neighbors;
    getNeighbors(new_state, neighbors);
    assert(!neighbors.empty());
    vector<shared_ptr<LLNode<DIM>>> new_nodes = chooseParent(new_state, neighbors, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    rewire(new_nodes, neighbors);

    // check goal
    for (auto &new_node : new_nodes) {
      if (calculateDistance(new_node->state, goal_state) < env.epsilon) {
        if (goal_node->interval.first <= new_node->earliest_arrival_time &&
            new_node->earliest_arrival_time < goal_node->earliest_arrival_time) {
          assert(goal_node->interval.first == new_node->interval.first);
          assert(goal_node->interval.second == new_node->interval.second);
          goal_node = new_node;
          best_arrival_time = goal_node->earliest_arrival_time;
        }
      } else {
        assert(calculateDistance(new_node->state, goal_state) >= env.epsilon);
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

template<int DIM>
State<DIM> SIRRT<DIM>::generateRandomState() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return goal_state;
  }

  State<DIM> random_state;
  for (int i = 0; i < DIM; ++i) {
    random_state[i] = spatial_distributions[i](env.gen);
  }
  return random_state;
}

template<int DIM>
shared_ptr<LLNode<DIM>> SIRRT<DIM>::getNearestNode(const State<DIM> &state) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::infinity();
  shared_ptr<LLNode<DIM>> nearest_node = nullptr;

  for (const auto &node : nodes) {
    const double distance = calculateDistance(node->state, state);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

template<int DIM>
State<DIM> SIRRT<DIM>::steer(const shared_ptr<LLNode<DIM>> &from_node, const State<DIM> &random_state,
                              SafeIntervalTable<DIM> &safe_interval_table) const {
  const double distance_to_random = calculateDistance(from_node->state, random_state);
  const double expand_distance = min(env.max_expand_distances[agent_id], distance_to_random);

  // 방향 벡터 계산
  State<DIM> direction = random_state - from_node->state;
  State<DIM> to_state = from_node->state;

  if (distance_to_random > 1e-9) {
    // 정규화된 방향으로 expand_distance만큼 이동
    double scale = expand_distance / distance_to_random;
    for (int i = 0; i < DIM; ++i) {
      to_state[i] += direction[i] * scale;
    }
  }

  if (constraint_table.obstacleConstrained(agent_id, from_node->state, to_state, env.radii[agent_id])) {
    return getInvalidState();
  }

  if (calculateDistance(to_state, goal_state) < env.epsilon) {
    return goal_state;
  }

  if (safe_interval_table.table[to_state].empty()) {
    if (env.algorithm == "pp")
      constraint_table.getSafeIntervalTablePath(agent_id, to_state, env.radii[agent_id],
                                                safe_interval_table.table[to_state]);
    else if (env.algorithm == "cbs")
      constraint_table.getSafeIntervalTable(agent_id, to_state, env.radii[agent_id],
                                            safe_interval_table.table[to_state]);
    if (safe_interval_table.table[to_state].empty()) {
      return getInvalidState();
    }
  }

  return to_state;
}

template<int DIM>
Path<DIM> SIRRT<DIM>::updatePath(const shared_ptr<LLNode<DIM>> &goal_node) const {
  Path<DIM> path;
  shared_ptr<LLNode<DIM>> curr_node = goal_node;

  while (curr_node->parent != nullptr) {
    const auto prev_node = curr_node->parent;
    const auto prev_time = prev_node->earliest_arrival_time;
    const auto curr_time = curr_node->earliest_arrival_time;
    assert(prev_time < curr_time);

    const auto expand_time = calculateDistance(prev_node->state, curr_node->state) / env.max_velocities[agent_id];
    path.emplace_back(curr_node->state, curr_time);
    if (prev_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(prev_node->state, curr_time - expand_time);
    }
    curr_node = curr_node->parent;
  }
  path.emplace_back(curr_node->state, 0);
  reverse(path.begin(), path.end());

  if (calculateDistance(get<0>(path.front()), start_state) >= env.epsilon) {
    throw runtime_error("Start state is not correct!");
  }
  if (calculateDistance(get<0>(path.back()), goal_state) >= env.epsilon) {
    throw runtime_error("Goal state is not correct!");
  }

  return path;
}

template<int DIM>
void SIRRT<DIM>::getNeighbors(const State<DIM> &state, vector<shared_ptr<LLNode<DIM>>> &neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto &node : nodes) {
    const double distance = calculateDistance(node->state, state);
    if (distance < connection_radius) {
      if (constraint_table.obstacleConstrained(agent_id, node->state, state, env.radii[agent_id]))
        continue;
      neighbors.emplace_back(node);
    }
  }
}

template<int DIM>
vector<shared_ptr<LLNode<DIM>>> SIRRT<DIM>::chooseParent(const State<DIM> &new_state,
                                                         const vector<shared_ptr<LLNode<DIM>>> &neighbors,
                                                         SafeIntervalTable<DIM> &safe_interval_table) const {
  assert(!neighbors.empty());

  auto new_nodes = vector<shared_ptr<LLNode<DIM>>>();

  for (auto &safe_interval : safe_interval_table.table[new_state]) {
    auto new_node = make_shared<LLNode<DIM>>(new_state, safe_interval.first, safe_interval.second);
    if (new_node->interval.first >= best_arrival_time)
      continue;

    for (const auto &neighbor : neighbors) {
      if (neighbor->earliest_arrival_time >= new_node->earliest_arrival_time)
        continue;
      const double expand_time = calculateDistance(new_node->state, neighbor->state) / env.max_velocities[agent_id];
      const double lower_bound = neighbor->earliest_arrival_time + expand_time;
      const double upper_bound = neighbor->interval.second + expand_time;

      if (lower_bound >= best_arrival_time)
        continue;
      if (lower_bound >= new_node->interval.second)
        continue;
      if (upper_bound <= new_node->interval.first)
        continue;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->state, new_node->state, expand_time,
          max(new_node->interval.first, lower_bound),
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

template<int DIM>
void SIRRT<DIM>::rewire(const vector<shared_ptr<LLNode<DIM>>> &new_nodes,
                        const vector<shared_ptr<LLNode<DIM>>> &neighbors) {
  assert(!neighbors.empty());
  for (auto &new_node : new_nodes) {
    for (auto &neighbor : neighbors) {
      if (neighbor->interval.first >= best_arrival_time)
        continue;
      if (new_node->earliest_arrival_time >= neighbor->earliest_arrival_time)
        continue;
      const double expand_time = calculateDistance(neighbor->state, new_node->state) / env.max_velocities[agent_id];
      const double lower_bound = new_node->earliest_arrival_time + expand_time;
      const double upper_bound = new_node->interval.second + expand_time;

      if (lower_bound >= best_arrival_time)
        continue;
      if (lower_bound >= neighbor->interval.second)
        continue;
      if (upper_bound <= neighbor->interval.first)
        continue;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, new_node->state, neighbor->state, expand_time,
          max(neighbor->interval.first, lower_bound),
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

template<int DIM>
void SIRRT<DIM>::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
  best_arrival_time = numeric_limits<double>::infinity();
}

// 명시적 템플릿 인스턴스화 (필요한 차원들에 대해)
template class SIRRT<2>;
template class SIRRT<3>;