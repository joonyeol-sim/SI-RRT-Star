#ifndef HLNODE_H
#define HLNODE_H

#include "common.h"

template<int DIM = DEFAULT_DIM>
class HLNode {
public:
  Solution<DIM> solution;
  vector<Conflict<DIM>> conflicts;
  vector<vector<Constraint<DIM>>> constraint_table;
  double cost;

  // 기본 생성자
  HLNode() : cost(0.0) {}

  // 비용으로 초기화하는 생성자
  explicit HLNode(double cost) : cost(cost) {}

  // 솔루션과 함께 초기화하는 생성자
  explicit HLNode(const Solution<DIM> &sol) : solution(sol), cost(0.0) {}

  // 모든 매개변수로 초기화하는 생성자
  HLNode(const Solution<DIM> &sol, const vector<Conflict<DIM>> &conf,
         const vector<vector<Constraint<DIM>>> &constraints, double c)
      : solution(sol), conflicts(conf), constraint_table(constraints), cost(c) {}
};

// 2D 특화 타입 별명 (하위 호환성)
using HLNode2D = HLNode<2>;

#endif // HLNODE_H