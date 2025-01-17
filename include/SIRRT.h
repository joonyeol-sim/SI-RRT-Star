#ifndef SIRRT_H
#define SIRRT_H

#include "ConstraintTable.h"
#include "LLNode.h"
#include "SafeIntervalTable.h"
#include "common.h"

class SIRRT {
public:
  std::uniform_real_distribution<> dis_width;
  std::uniform_real_distribution<> dis_height;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode>> nodes;
  shared_ptr<LLNode> goal_node;
  Point start_point;
  Point goal_point;
  Path path;
  int agent_id;
  SharedEnv &env;
  ConstraintTable &constraint_table;
  double best_arrival_time = numeric_limits<double>::infinity();

  SIRRT(int agent_id, SharedEnv &env, ConstraintTable &constraint_table)
      : dis_width(env.radii[agent_id], env.width - env.radii[agent_id]),
        dis_height(env.radii[agent_id], env.height - env.radii[agent_id]), dis_100(0.0, 100.0), env(env),
        constraint_table(constraint_table), agent_id(agent_id), start_point(env.start_points[agent_id]),
        goal_point(env.goal_points[agent_id]) {}
  ~SIRRT() = default;
  Path run();
  Point generateRandomPoint();
  shared_ptr<LLNode> getNearestNode(const Point &point) const;
  Point steer(const shared_ptr<LLNode> &from_node, const Point &random_point,
              SafeIntervalTable &safe_interval_table) const;
  Path updatePath(const shared_ptr<LLNode> &goal_node) const;
  void getNeighbors(Point point, vector<shared_ptr<LLNode>> &neighbors) const;
  vector<shared_ptr<LLNode>> chooseParent(const Point &new_point, const vector<shared_ptr<LLNode>> &neighbors,
                                          SafeIntervalTable &safe_interval_table) const;
  void rewire(const vector<shared_ptr<LLNode>> &new_nodes, const vector<shared_ptr<LLNode>> &neighbors);
  void release();
};

#endif // SIRRT_H
