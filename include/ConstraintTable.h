#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include "SharedEnv.h"
#include "common.h"

template<int DIM = DEFAULT_DIM>
class ConstraintTable {
public:
  vector<Path<DIM>> path_table;
  vector<vector<Constraint<DIM>>> hard_constraint_table;
  vector<vector<Constraint<DIM>>> soft_constraint_table;
  SharedEnv<DIM> &env;

  explicit ConstraintTable(SharedEnv<DIM> &env)
      : env(env), path_table(env.num_of_robots), hard_constraint_table(env.num_of_robots),
        soft_constraint_table(env.num_of_robots) {}

  bool obstacleConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double radius) const;

  bool pathConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double from_time, double to_time,
                       double radius) const;

  bool hardConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double from_time, double to_time,
                       double radius) const;

  bool targetConstrained(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double from_time, double to_time,
                         double radius) const;

  void getSafeIntervalTablePath(int agent_id, const State<DIM> &to_state, double radius,
                                vector<Interval> &safe_intervals) const;

  void getSafeIntervalTable(int agent_id, const State<DIM> &to_state, double radius, vector<Interval> &safe_intervals) const;

  double getEarliestArrivalTime(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double expand_time,
                                double lower_bound, double upper_bound, double radius) const;

  void insertCollisionIntervalToSIT(vector<Interval> &safe_intervals, double t_min, double t_max) const;

  void interpolateState(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state,
                        vector<State<DIM>> &interpolated_states) const;

  void interpolateStateTime(int agent_id, const State<DIM> &from_state, const State<DIM> &to_state, double from_time,
                            double to_time, vector<State<DIM>> &interpolated_states, vector<double> &interpolated_times) const;

  bool checkConflicts(const Solution<DIM> &solution) const;

  double TimeToCollision(const State<DIM> &start_state1, double radius1, Velocity<DIM> &v1, const State<DIM> &start_state2,
                         double radius2, Velocity<DIM> &v2, double from_time2, double to_time2) const;
};
#endif // CONSTRAINTTABLE_H