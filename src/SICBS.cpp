#include "SICBS.h"

template<int DIM>
Solution<DIM> SICBS<DIM>::run() {
  HLNode<DIM> root;
  root.constraint_table.resize(env.num_of_robots);
  root.solution = getInitialSolution();
  if (root.solution.empty()) {
    cout << "No solution" << endl;
    return {};
  }
  root.cost = calculateCost(root.solution);
  findConflicts(root.solution, root.conflicts);

  open_list.push(root);
  while (!open_list.empty()) {
    auto curr_node = open_list.top();
    // cout << "Cost : " << curr_node.conflicts.size() << endl;
    open_list.pop();

    if (curr_node.conflicts.empty()) {
      sum_of_costs = curr_node.cost;
      for (const auto &path : curr_node.solution) {
        makespan = max(makespan, get<1>(path.back()));
      }
      return curr_node.solution;
    }

    auto conflict = curr_node.conflicts[0];
    vector<int> agent_ids = {get<0>(conflict), get<1>(conflict)};
    // cout << "Conflict : "
    //      << "(Agent " << agent_ids[0] << ", Agent " << agent_ids[1] << "), " << endl;
    const auto partial_path1 = get<0>(get<2>(conflict));
    const auto partial_path2 = get<1>(get<2>(conflict));
    // cout << "Interval : " << get<1>(partial_path1.front()) << " ~ " << get<1>(partial_path1.back()) << endl;
    // print partial path (n차원 지원)
    // cout << "partial path" << agent_ids[0] << " : ";
    // for (const auto& state_time : partial_path1) {
    //   const auto& state = get<0>(state_time);
    //   cout << "(";
    //   for (int d = 0; d < DIM; ++d) {
    //     cout << state[d];
    //     if (d < DIM - 1) cout << ",";
    //   }
    //   cout << "," << get<1>(state_time) << ")->";
    // }
    // cout << endl;
    // cout << "partial path" << agent_ids[1] << " : ";
    // for (const auto& state_time : partial_path2) {
    //   const auto& state = get<0>(state_time);
    //   cout << "(";
    //   for (int d = 0; d < DIM; ++d) {
    //     cout << state[d];
    //     if (d < DIM - 1) cout << ",";
    //   }
    //   cout << "," << get<1>(state_time) << ")->";
    // }
    // cout << endl;
    vector<Path<DIM>> partial_paths = {partial_path1, partial_path2};
    for (int i = 0; i < agent_ids.size(); i++) {
      const int j = (i + 1) % 2;

      // copy curr_node
      auto new_node = curr_node;

      // update constraints
      new_node.constraint_table[agent_ids[i]].emplace_back(env.radii[agent_ids[j]], partial_paths[j]);
      constraint_table.hard_constraint_table = new_node.constraint_table;

      // print before path (n차원 지원)
      // cout << "Before path" << agent_ids[i] << ": ";
      // for (const auto& state_time : new_node.solution[agent_ids[i]]) {
      //   const auto& state = get<0>(state_time);
      //   cout << "(";
      //   for (int d = 0; d < DIM; ++d) {
      //     cout << state[d];
      //     if (d < DIM - 1) cout << ",";
      //   }
      //   cout << "," << get<1>(state_time) << ")->";
      // }
      // cout << endl;

      // update path
      // cout << "Planning for agent " << agent_ids[i] << endl;
      new_node.solution[agent_ids[i]] = low_level_planners[agent_ids[i]].run();
      if (new_node.solution[agent_ids[i]].empty())
        continue;
      // constraint_table.updateSoftConstraint(i, new_node.solution[agent_ids[i]]);

      // print after path (n차원 지원)
      // cout << "After path" << agent_ids[i] << ": ";
      // for (const auto& state_time : new_node.solution[agent_ids[i]]) {
      //   const auto& state = get<0>(state_time);
      //   cout << "(";
      //   for (int d = 0; d < DIM; ++d) {
      //     cout << state[d];
      //     if (d < DIM - 1) cout << ",";
      //   }
      //   cout << "," << get<1>(state_time) << ")->";
      // }
      // cout << endl;

      // update cost and conflicts
      new_node.cost = calculateCost(new_node.solution);
      new_node.conflicts.clear();
      findConflicts(new_node.solution, new_node.conflicts);

      // push new node to open list
      open_list.push(new_node);
    }
  }
  cout << "No solution" << endl;
  return {};
}

template<int DIM>
Solution<DIM> SICBS<DIM>::getInitialSolution() {
  Solution<DIM> solution(env.num_of_robots);
  std::vector<std::thread> threads(env.num_of_robots);

  auto plan_path = [&](int agent_id) {
    auto path = low_level_planners[agent_id].run();
    // std::cout << "Agent " << agent_id << " found a path" << std::endl;
    solution[agent_id] = path;
  };

  for (int agent_id = 0; agent_id < env.num_of_robots; ++agent_id) {
    threads[agent_id] = std::thread(plan_path, agent_id);
  }

  for (auto &thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  return solution;
}

template<int DIM>
double SICBS<DIM>::calculateCost(const Solution<DIM> &solution) {
  double cost = 0.0;
  for (const auto &path : solution) {
    cost += get<1>(path.back());
  }
  return cost;
}

template<int DIM>
void SICBS<DIM>::findConflicts(const Solution<DIM> &solution, vector<Conflict<DIM>> &conflicts) const {
  for (int agent1_id = 0; agent1_id < env.num_of_robots; ++agent1_id) {
    for (int agent2_id = agent1_id + 1; agent2_id < env.num_of_robots; ++agent2_id) {
      Path<DIM> partial_path1 = {};
      Path<DIM> partial_path2 = {};

      auto index1 = 0;
      auto index2 = 0;
      auto [prev_point1, prev_time1] = solution[agent1_id][index1];
      auto [next_point1, next_time1] = solution[agent1_id][index1 + 1];
      auto [prev_point2, prev_time2] = solution[agent2_id][index2];
      auto [next_point2, next_time2] = solution[agent2_id][index2 + 1];

      const double max_time = max(get<1>(solution[agent1_id].back()), get<1>(solution[agent2_id].back()));
      bool is_safe = true;
      double curr_time = 0.0;

      while (curr_time < max_time) {
        while (curr_time >= next_time1 && index1 + 2 < solution[agent1_id].size()) {
          if (!is_safe) {
            partial_path1.emplace_back(make_tuple(next_point1, next_time1));
          }
          index1++;
          prev_point1 = get<0>(solution[agent1_id][index1]);
          prev_time1 = get<1>(solution[agent1_id][index1]);
          next_point1 = get<0>(solution[agent1_id][index1 + 1]);
          next_time1 = get<1>(solution[agent1_id][index1 + 1]);
        }
        while (curr_time >= next_time2 && index2 + 2 < solution[agent2_id].size()) {
          if (!is_safe) {
            partial_path2.emplace_back(make_tuple(next_point2, next_time2));
          }
          index2++;
          prev_point2 = get<0>(solution[agent2_id][index2]);
          prev_time2 = get<1>(solution[agent2_id][index2]);
          next_point2 = get<0>(solution[agent2_id][index2 + 1]);
          next_time2 = get<1>(solution[agent2_id][index2 + 1]);
        }
        auto agent1_expand_time = curr_time - prev_time1;
        auto agent2_expand_time = curr_time - prev_time2;

        if (curr_time >= next_time1) {
          agent1_expand_time = next_time1 - prev_time1;
        }
        if (curr_time >= next_time2) {
          agent2_expand_time = next_time2 - prev_time2;
        }
        assert(agent1_expand_time >= 0.0);
        assert(agent2_expand_time >= 0.0);

        // n차원 방향 벡터 계산
        Velocity<DIM> agent1_direction = toVelocity<DIM>(prev_point1, next_point1);
        Velocity<DIM> agent2_direction = toVelocity<DIM>(prev_point2, next_point2);
        
        // 방향 벡터 정규화 및 최대 속도 적용
        auto agent1_point = prev_point1;
        if (agent1_direction.length() > 0.0) {
          Velocity<DIM> normalized_dir1 = agent1_direction.normalize();
          Velocity<DIM> movement1 = normalized_dir1 * (env.max_velocities[agent1_id] * agent1_expand_time);
          agent1_point = prev_point1 + toState<DIM>(movement1);
        }
        
        auto agent2_point = prev_point2;
        if (agent2_direction.length() > 0.0) {
          Velocity<DIM> normalized_dir2 = agent2_direction.normalize();
          Velocity<DIM> movement2 = normalized_dir2 * (env.max_velocities[agent2_id] * agent2_expand_time);
          agent2_point = prev_point2 + toState<DIM>(movement2);
        }

        if (is_safe && calculateDistance(agent1_point, agent2_point) < env.radii[agent1_id] + env.radii[agent2_id]) {
          is_safe = false;
          partial_path1.emplace_back(make_tuple(agent1_point, curr_time));
          partial_path2.emplace_back(make_tuple(agent2_point, curr_time));
        } else if (!is_safe &&
                   calculateDistance(agent1_point, agent2_point) >= env.radii[agent1_id] + env.radii[agent2_id]) {
          is_safe = true;
          assert(curr_time >= get<1>(partial_path1.back()));
          assert(curr_time >= get<1>(partial_path2.back()));
          if (get<1>(partial_path1.back()) != curr_time) {
            partial_path1.emplace_back(make_tuple(agent1_point, curr_time));
          }
          if (get<1>(partial_path2.back()) != curr_time) {
            partial_path2.emplace_back(make_tuple(agent2_point, curr_time));
          }
          conflicts.emplace_back(agent1_id, agent2_id, make_tuple(partial_path1, partial_path2));
          partial_path1.clear();
          partial_path2.clear();
        }
        curr_time += env.check_time_resolution;
      }
    }
  }
}

// 명시적 템플릿 인스턴스화
template class SICBS<2>;
template class SICBS<3>;
