#ifndef SHAREDENV_H
#define SHAREDENV_H

#include "common.h"

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
  int width;
  int height;
  double v_max = 5.0;
  double a_max = 1.0;
  vector<double> radii;
  vector<Point> start_points;
  vector<Point> goal_points;
  vector<shared_ptr<Obstacle>> obstacles;
  // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  unsigned seed = 0;
  default_random_engine gen;
  string algorithm;

  SharedEnv(int num_of_robots, int width, int height, const vector<Point> &start_points,
            const vector<Point> &goal_points, const vector<double> &radii, const vector<double> &max_expand_distances,
            const vector<double> &max_velocities, const vector<int> &iterations,
            const vector<double> &goal_sample_rates, const vector<shared_ptr<Obstacle>> &obstacles, string algorithm)
      : num_of_robots(num_of_robots), width(width), height(height), start_points(start_points),
        goal_points(goal_points), radii(radii), max_expand_distances(max_expand_distances),
        max_velocities(max_velocities), iterations(iterations), goal_sample_rates(goal_sample_rates),
        obstacles(obstacles), algorithm(std::move(algorithm)), gen(seed) {}

  void generateRandomInstance() {
    start_points.clear();
    goal_points.clear();

    int agent_id = 0;
    while (start_points.size() < num_of_robots) {
      uniform_real_distribution<> dis_width(radii[agent_id], width - radii[agent_id]);
      uniform_real_distribution<> dis_height(radii[agent_id], height - radii[agent_id]);
      auto start_point = Point(dis_width(gen), dis_height(gen));
      if (!obstacleConstrained(start_point, radii[agent_id]) && !occupied(start_point, radii[agent_id], start_points)) {
        start_points.emplace_back(start_point);
        agent_id++;
      }
    }

    agent_id = 0;
    while (goal_points.size() < num_of_robots) {
      uniform_real_distribution<> dis_width(radii[agent_id], width - radii[agent_id]);
      uniform_real_distribution<> dis_height(radii[agent_id], height - radii[agent_id]);
      auto goal_point = Point(dis_width(gen), dis_height(gen));
      if (!obstacleConstrained(goal_point, radii[agent_id]) && !occupied(goal_point, radii[agent_id], goal_points)) {
        goal_points.emplace_back(goal_point);
        agent_id++;
      }
    }
  }

  bool obstacleConstrained(const Point &other_point, const double other_radius) const {
    return any_of(obstacles.begin(), obstacles.end(), [&](const shared_ptr<Obstacle> &obstacle) {
      return obstacle->constrained(other_point, other_radius);
    });
  }

  bool occupied(const Point &other_point, const double other_radius, const vector<Point> &other_points) const {
    for (int agent_id = 0; agent_id < other_points.size(); ++agent_id) {
      if (calculateDistance(other_point, other_points[agent_id]) < (radii[agent_id] + other_radius) * 2) {
        return true;
      }
    }
    return false;
  }
};

#endif // SHAREDENV_H
