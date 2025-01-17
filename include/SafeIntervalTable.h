#ifndef SAFEINTERVALTABLE_H
#define SAFEINTERVALTABLE_H

#include "SharedEnv.h"
#include "common.h"

class SafeIntervalTable {
public:
  unordered_map<Point, vector<Interval>, PointHash> table;
  SharedEnv &env;
  explicit SafeIntervalTable(SharedEnv &env) : env(env) {}
};
#endif // SAFEINTERVALTABLE_H
