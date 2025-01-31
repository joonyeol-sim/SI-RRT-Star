#ifndef HLNODE_H
#define HLNODE_H

#include "common.h"

class HLNode {
public:
  Solution solution;
  vector<Conflict> conflicts;
  vector<vector<Constraint>> constraint_table;
  double cost;
};

#endif // HLNODE_H
