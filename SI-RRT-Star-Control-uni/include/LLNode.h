#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
public:
  Point point;
  double angle;
  Velocity velocity;
  // 이 노드까지 어떤 control로 왔는지 저장
  Control linear_acceleration;
  Control angular_acceleration;

  shared_ptr<LLNode> parent;
  double earliest_arrival_time;
  Interval interval;
  explicit LLNode(Point point, double angle, Velocity velocity, double lower_bound, double upper_bound)
      : point(point), angle(angle), velocity(velocity), earliest_arrival_time(numeric_limits<double>::infinity()),
        interval(lower_bound, upper_bound), linear_acceleration(0.0, 0.0, 0.0, 0.0),
        angular_acceleration(0.0, 0.0, 0.0, 0.0) {}
};

#endif // LLNODE_H
