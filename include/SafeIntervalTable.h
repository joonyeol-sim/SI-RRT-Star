#ifndef SAFEINTERVALTABLE_H
#define SAFEINTERVALTABLE_H

#include "SharedEnv.h"
#include "common.h"

template<int DIM = DEFAULT_DIM>
class SafeIntervalTable {
public:
  unordered_map<State<DIM>, vector<Interval>, StateHash<DIM>> table;
  SharedEnv<DIM> &env;

  explicit SafeIntervalTable(SharedEnv<DIM> &env) : env(env) {}
};

// 2D 특화 타입 별명 (하위 호환성)
using SafeIntervalTable2D = SafeIntervalTable<2>;

#endif // SAFEINTERVALTABLE_H