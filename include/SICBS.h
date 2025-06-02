#ifndef SICBS_H
#define SICBS_H

#include "ConstraintTable.h"
#include "HLNode.h"
#include "SIRRT.h"
#include "SharedEnv.h"
#include "common.h"

template<int DIM = DEFAULT_DIM>
struct compare_function {
  bool operator()(const HLNode<DIM> &a, const HLNode<DIM> &b) const {
    if (a.conflicts.size() == b.conflicts.size()) {
      return a.cost > b.cost;
    }
    return a.conflicts.size() > b.conflicts.size();
  }
};

template<int DIM = DEFAULT_DIM>
class SICBS {
public:
  boost::heap::fibonacci_heap<HLNode<DIM>, boost::heap::compare<compare_function<DIM>>> open_list;
  vector<shared_ptr<HLNode<DIM>>> nodes;
  SharedEnv<DIM> &env;
  ConstraintTable<DIM> &constraint_table;
  vector<SIRRT<DIM>> low_level_planners;
  double sum_of_costs = 0.0;
  double makespan = 0.0;

  SICBS(SharedEnv<DIM> &env, ConstraintTable<DIM> &constraint_table) 
    : env(env), constraint_table(constraint_table) {
    low_level_planners.reserve(env.num_of_robots);
    for (int i = 0; i < env.num_of_robots; i++) {
      low_level_planners.emplace_back(i, env, constraint_table);
    }
  }
  ~SICBS() = default;
  Solution<DIM> run();
  Solution<DIM> getInitialSolution();
  static double calculateCost(const Solution<DIM> &solution);
  void getConflicts(const Solution<DIM> &solution, vector<Conflict<DIM>> &conflicts) const;
  void findConflicts(const Solution<DIM> &solution, vector<Conflict<DIM>> &conflicts) const;
};

// 2D 특화 타입 별명 (하위 호환성)
using SICBS2D = SICBS<2>;

#endif // SICBS_H
