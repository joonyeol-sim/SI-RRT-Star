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
  // 이 노드까지 어떤 control로 왔는지 저장
  Control x_control;
  Control y_control;

  explicit LLNode(Point point, Velocity velocity, double lower_bound, double upper_bound)
      : point(point), velocity(velocity), earliest_arrival_time(numeric_limits<double>::infinity()),
        interval(lower_bound, upper_bound), x_control(0.0, 0.0, 0.0, 0.0), y_control(0.0, 0.0, 0.0, 0.0) {}
};

#endif // LLNODE_H
