#ifndef SICBS_H
#define SICBS_H

#include "ConstraintTable.h"
#include "HLNode.h"
#include "SIRRT.h"
#include "SharedEnv.h"
#include "common.h"

struct compare_function {
  // bool operator()(const HLNode& a, const HLNode& b) const {
  //   if (a.cost == b.cost) {
  //     return a.conflicts.size() > b.conflicts.size();
  //   }
  //   return a.cost > b.cost;
  // }
  bool operator()(const HLNode &a, const HLNode &b) const {
    if (a.conflicts.size() == b.conflicts.size()) {
      return a.cost > b.cost;
    }
    return a.conflicts.size() > b.conflicts.size();
  }
};

class SICBS {
public:
  boost::heap::fibonacci_heap<HLNode, boost::heap::compare<compare_function>> open_list;
  vector<shared_ptr<HLNode>> nodes;
  SharedEnv &env;
  ConstraintTable &constraint_table;
  vector<SIRRT> low_level_planners;
  double sum_of_costs = 0.0;
  double makespan = 0.0;

  SICBS(SharedEnv &env, ConstraintTable &constraint_table) : env(env), constraint_table(constraint_table) {
    low_level_planners.reserve(env.num_of_robots);
    for (int i = 0; i < env.num_of_robots; i++) {
      low_level_planners.emplace_back(i, env, constraint_table);
    }
  }
  ~SICBS() = default;
  PathSolution run();
  PathSolution getInitialSolution();
  static double calculateCost(const PathSolution &solution);
  void getConflicts(const PathSolution &solution, vector<Conflict> &conflicts) const;
  void findConflicts(const PathSolution &solution, vector<Conflict> &conflicts) const;
};

#endif // SICBS_H
