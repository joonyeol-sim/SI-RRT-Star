#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include "SharedEnv.h"
#include "common.h"

class ConstraintTable {
public:
  vector<Path> path_table;
  vector<Trajectory> trajectory_table;
  vector<vector<Constraint>> hard_constraint_table;
  vector<vector<Constraint>> soft_constraint_table;
  SharedEnv &env;

  ConstraintTable(SharedEnv &env)
      : env(env), path_table(env.num_of_robots), trajectory_table(env.num_of_robots),
        hard_constraint_table(env.num_of_robots), soft_constraint_table(env.num_of_robots) {}

  bool obstacleConstrained(int agent_id, const Point &from_point, const Velocity &from_velocity, double from_time,
                           double to_time, const Control &x_control, const Control &y_control,
                           double agent_radius) const;

  bool pathConstrained(int agent_id, const Point &from_point, Velocity from_velocity, double from_time, double to_time,
                       Control &x_control, Control &y_control, double radius) const;

  bool hardConstrained(int agent_id, const Point &from_point, const Point &to_point, double from_time, double to_time,
                       double radius) const;

  bool targetConstrained(int agent_id, const Point &from_point, const Velocity &from_velocity, double from_time,
                         double to_time, const Control &x_control, const Control &y_control, double radius) const;

  void getSafeIntervalTablePath(int agent_id, const Point &to_point, double radius,
                                vector<Interval> &safe_intervals) const;

  void getSafeIntervalTable(int agent_id, const Point &to_point, double radius, vector<Interval> &safe_intervals) const;

  optional<XYControl> getEarliestArrivalControl(int agent_id, const Point &from_point, const Point &to_point,
                                                const Velocity &from_velocity, const Velocity &to_velocity,
                                                double from_time, double arrival_lower_bound,
                                                double arrival_upper_bound, double radius) const;

  void insertCollisionIntervalToSIT(vector<Interval> &safe_intervals, double t_min, double t_max) const;

  bool checkConflicts(const PathSolution &solution) const;

  double TimeToCollision(const Point &start_point1, double radius1, Velocity &v1, const Point &start_point2,
                         double radius2, Velocity &v2, double from_time2, double to_time2) const;
};
#endif // CONSTRAINTTABLE_H
