#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
public:
  Point point;
  Velocity velocity;
  shared_ptr<LLNode> parent;
  double earliest_arrival_time;
  Interval interval;

  explicit LLNode(Point point, Velocity velocity, double lower_bound, double upper_bound)
      : point(point), velocity(velocity), earliest_arrival_time(numeric_limits<double>::infinity()),
        interval(lower_bound, upper_bound) {}
};

#endif // LLNODE_H
