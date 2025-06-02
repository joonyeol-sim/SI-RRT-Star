#ifndef SIRRT_H
#define SIRRT_H

#include "ConstraintTable.h"
#include "LLNode.h"
#include "SafeIntervalTable.h"
#include "common.h"

template<int DIM = DEFAULT_DIM>
class SIRRT {
public:
  std::array<std::uniform_real_distribution<>, DIM> spatial_distributions;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode<DIM>>> nodes;
  shared_ptr<LLNode<DIM>> goal_node;
  State<DIM> start_state;
  State<DIM> goal_state;
  Path<DIM> path;
  int agent_id;
  SharedEnv<DIM> &env;
  ConstraintTable<DIM> &constraint_table;
  double best_arrival_time = numeric_limits<double>::infinity();

  SIRRT(int agent_id, SharedEnv<DIM> &env, ConstraintTable<DIM> &constraint_table)
      : dis_100(0.0, 100.0), env(env), constraint_table(constraint_table), agent_id(agent_id),
        start_state(env.start_states[agent_id]), goal_state(env.goal_states[agent_id]) {
    // N차원 공간 분포 초기화
    for (int i = 0; i < DIM; ++i) {
      spatial_distributions[i] = std::uniform_real_distribution<>(
          env.radii[agent_id], env.bounds[i] - env.radii[agent_id]);
    }
  }

  ~SIRRT() = default;

  Path<DIM> run();
  State<DIM> generateRandomState();
  shared_ptr<LLNode<DIM>> getNearestNode(const State<DIM> &state) const;
  State<DIM> steer(const shared_ptr<LLNode<DIM>> &from_node, const State<DIM> &random_state,
                   SafeIntervalTable<DIM> &safe_interval_table) const;
  Path<DIM> updatePath(const shared_ptr<LLNode<DIM>> &goal_node) const;
  void getNeighbors(const State<DIM> &state, vector<shared_ptr<LLNode<DIM>>> &neighbors) const;
  vector<shared_ptr<LLNode<DIM>>> chooseParent(const State<DIM> &new_state,
                                               const vector<shared_ptr<LLNode<DIM>>> &neighbors,
                                               SafeIntervalTable<DIM> &safe_interval_table) const;
  void rewire(const vector<shared_ptr<LLNode<DIM>>> &new_nodes,
              const vector<shared_ptr<LLNode<DIM>>> &neighbors);
  void release();

private:
  // 유효하지 않은 상태를 나타내는 헬퍼 함수
  State<DIM> getInvalidState() const {
    State<DIM> invalid_state;
    for (int i = 0; i < DIM; ++i) {
      invalid_state[i] = -1.0;
    }
    return invalid_state;
  }

  // 상태가 유효한지 확인하는 헬퍼 함수
  bool isValidState(const State<DIM> &state) const {
    for (int i = 0; i < DIM; ++i) {
      if (state[i] < 0.0) return false;
    }
    return true;
  }
};

// 2D 특화 타입 별명 (하위 호환성)
using SIRRT2D = SIRRT<2>;

#endif // SIRRT_H