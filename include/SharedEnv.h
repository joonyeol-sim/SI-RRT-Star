
#ifndef SHAREDENV_H
#define SHAREDENV_H

#include "common.h"

template<int DIM = DEFAULT_DIM>
class SharedEnv {
public:
  vector<double> max_expand_distances;
  vector<double> max_velocities;
  const double epsilon = 0.001;
  double time_resolution = 1.0;
  double check_time_resolution = 0.5;
  vector<int> iterations;
  vector<double> goal_sample_rates;
  int num_of_robots;
  std::array<int, DIM> bounds; // width, height, depth, etc.
  vector<double> radii;
  vector<State<DIM>> start_states;
  vector<State<DIM>> goal_states;
  vector<shared_ptr<Obstacle<DIM>>> obstacles;
  unsigned seed = 0;
  default_random_engine gen;
  string algorithm;

  // 생성자
  SharedEnv(int num_of_robots, const std::array<int, DIM> &bounds,
            const vector<State<DIM>> &start_states, const vector<State<DIM>> &goal_states,
            const vector<double> &radii, const vector<double> &max_expand_distances,
            const vector<double> &max_velocities, const vector<int> &iterations,
            const vector<double> &goal_sample_rates,
            const vector<shared_ptr<Obstacle<DIM>>> &obstacles, string algorithm)
      : num_of_robots(num_of_robots), bounds(bounds), start_states(start_states),
        goal_states(goal_states), radii(radii), max_expand_distances(max_expand_distances),
        max_velocities(max_velocities), iterations(iterations), goal_sample_rates(goal_sample_rates),
        obstacles(obstacles), algorithm(std::move(algorithm)), gen(seed) {}

  // 2D 호환성을 위한 생성자 (2D에서만 사용 가능)
  SharedEnv(int num_of_robots, int width, int height, int depth, const vector<State<DIM>> &start_states,
            const vector<State<DIM>> &goal_states, const vector<double> &radii,
            const vector<double> &max_expand_distances, const vector<double> &max_velocities,
            const vector<int> &iterations, const vector<double> &goal_sample_rates,
            const vector<shared_ptr<Obstacle<DIM>>> &obstacles, string algorithm)
      : num_of_robots(num_of_robots), start_states(start_states),
        goal_states(goal_states), radii(radii), max_expand_distances(max_expand_distances),
        max_velocities(max_velocities), iterations(iterations), goal_sample_rates(goal_sample_rates),
        obstacles(obstacles), algorithm(std::move(algorithm)), gen(seed) {
    static_assert(DIM >= 2, "This constructor requires at least 2D");
    bounds[0] = width;
    bounds[1] = height;
    bounds[2] = depth;
  }

  // 편의를 위한 getter 함수들
  int width() const {
    static_assert(DIM >= 1, "Need at least 1D for width");
    return bounds[0];
  }

  int height() const {
    static_assert(DIM >= 2, "Need at least 2D for height");
    return bounds[1];
  }

  int depth() const {
    static_assert(DIM >= 3, "Need at least 3D for depth");
    return bounds[2];
  }

  void generateRandomInstance() {
    start_states.clear();
    goal_states.clear();

    int agent_id = 0;
    while (start_states.size() < num_of_robots) {
      State<DIM> start_state = generateRandomState(agent_id);
      if (!obstacleConstrained(start_state, radii[agent_id]) &&
          !occupied(start_state, radii[agent_id], start_states)) {
        start_states.emplace_back(start_state);
        agent_id++;
          }
    }

    agent_id = 0;
    while (goal_states.size() < num_of_robots) {
      State<DIM> goal_state = generateRandomState(agent_id);
      if (!obstacleConstrained(goal_state, radii[agent_id]) &&
          !occupied(goal_state, radii[agent_id], goal_states)){  // 시작점들과도 겹치지 않는지 확인
        goal_states.emplace_back(goal_state);
        agent_id++;}
    }
  }

  bool obstacleConstrained(const State<DIM> &state, const double radius) const {
    return any_of(obstacles.begin(), obstacles.end(),
                  [&](const shared_ptr<Obstacle<DIM>> &obstacle) {
                    return obstacle->isColliding(state, radius);
                  });
  }

  bool occupied(const State<DIM> &state, const double radius,
                const vector<State<DIM>> &other_states) const {
    for (int agent_id = 0; agent_id < other_states.size(); ++agent_id) {
      if (calculateDistance(state, other_states[agent_id]) < (radii[agent_id] + radius) * 2) {
        return true;
      }
    }
    return false;
  }

private:
  State<DIM> generateRandomState(int agent_id) {
    State<DIM> state;
    for (int i = 0; i < DIM; ++i) {
      uniform_real_distribution<> dis(radii[agent_id], bounds[i] - radii[agent_id]);
      state[i] = dis(gen);
    }
    return state;
  }
};

// 2D 특화 타입 별명 (하위 호환성)
using SharedEnv2D = SharedEnv<2>;

#endif // SHAREDENV_H