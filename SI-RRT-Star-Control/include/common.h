#ifndef COMMON_H
#define COMMON_H

#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

using namespace std;

class Velocity {
public:
  double x, y;

  explicit Velocity(const double vx = 0.0, const double vy = 0.0) : x(vx), y(vy) {}

  bool operator==(const Velocity &other) const { return x == other.x && y == other.y; }

  bool operator!=(const Velocity &other) const { return !(*this == other); }

  Velocity operator+(const Velocity &other) const { return Velocity(x + other.x, y + other.y); }

  Velocity operator-(const Velocity &other) const { return Velocity(x - other.x, y - other.y); }

  double dot(const Velocity &other) const { return x * other.x + y * other.y; }

  double length() const { return std::sqrt(x * x + y * y); }
};

class Point {
public:
  double x, y;

  explicit Point(const double x = 0.0, const double y = 0.0) : x(x), y(y) {}

  bool operator==(const Point &other) const { return x == other.x && y == other.y; }

  bool operator!=(const Point &other) const { return !(*this == other); }

  Point operator+(const Point &other) const { return Point(x + other.x, y + other.y); }

  Point operator-(const Point &other) const { return Point(x - other.x, y - other.y); }

  double dot(const Point &other) const { return x * other.x + y * other.y; }
  double dot(const Velocity &other) const { return x * other.x + y * other.y; }
};

class Control {
public:
  pair<double, double> control_first; // (a_acc, t_acc)
  pair<double, double> control_const; // (a_const, t_const)
  pair<double, double> control_last;  // (a_dec, t_dec)

  Control(double acc_first, double t_first, double acc_last, double t_last, double acc_const = 0.0,
          double t_const = 0.0)
      : control_first{acc_first, t_first}, control_last{acc_last, t_last}, control_const{acc_const, t_const} {}

  virtual void print() const {
    cout << "acc_first: " << control_first.first << ", t_first: " << control_first.second << endl;
    cout << "acc_const: " << control_const.first << ", t_const: " << control_const.second << endl;
    cout << "acc_last: " << control_last.first << ", t_last: " << control_last.second << endl;
  }

  virtual ~Control() = default;
};

using Path = std::vector<std::tuple<Point, double>>;
using ControlPath = std::tuple<std::vector<Control>, std::vector<Control>>;
using Interval = std::pair<double, double>;
using Conflict = std::tuple<int, int, std::tuple<Path, Path>>;
using Constraint = std::tuple<double, Path>;
using Solution = std::vector<Path>;
using ControlSolution = std::vector<ControlPath>;

void openFile(ofstream &file, const string &filename);

void writePath(ofstream &file, const Path &path);

void savePath(const Path &path, const string &filename);

void saveSolution(const Solution &solution, const string &filename);

void saveControlSolution(const std::vector<ControlPath> &control_solution, const string &filename);

void saveData(double cost, double makespan, double duration, const string &filename);

double calculateDistance(Point point1, Point point2);

std::optional<Control> findControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max);

std::optional<Control> findControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max);

std::optional<Control> findControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T);

std::optional<Control> findControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T);

std::optional<Control> findConstControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max);

std::optional<Control> findConstControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max);

std::optional<Control> findConstControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T);

std::optional<Control> findConstControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T);

std::optional<double> calculateCostToGo(const Point &start, const Point &goal, const Velocity &v_start,
                                        const Velocity &v_goal, double v_max, double a_max);

std::optional<std::tuple<double, std::unique_ptr<Control>, std::unique_ptr<Control>>>
calculateCostToGoWithControls(const Point &start, const Point &goal, const Velocity &v_start, const Velocity &v_goal,
                              double v_max, double a_max);

struct PointHash {
  size_t operator()(const Point &point) const {
    auto [x, y] = point;
    size_t seed = 0;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    return seed;
  }
};

class Obstacle {
public:
  Point point;

  Obstacle(double x, double y) : point(Point(x, y)) {}

  virtual ~Obstacle() = default;

  virtual bool constrained(const Point &other_point, const double other_radius) = 0;
};

class RectangularObstacle : public Obstacle {
public:
  double width, height;

  RectangularObstacle(double x, double y, double width, double height) : Obstacle(x, y), width(width), height(height) {}

  bool constrained(const Point &other_point, const double other_radius) override {
    const auto &[agentX, agentY] = other_point;
    const double agentR = other_radius;
    const auto &[x, y] = point;

    double rectLeft = x - width / 2;
    double rectRight = x + width / 2;
    double rectTop = y - height / 2;
    double rectBottom = y + height / 2;

    if (agentX >= rectLeft && agentX <= rectRight && agentY >= rectTop && agentY <= rectBottom) {
      return true;
    }

    double closestX = (agentX <= rectLeft) ? rectLeft : (agentX >= rectRight) ? rectRight : agentX;
    double closestY = (agentY <= rectTop) ? rectTop : (agentY >= rectBottom) ? rectBottom : agentY;

    double distX = agentX - closestX;
    double distY = agentY - closestY;

    return (distX * distX + distY * distY) <= (agentR * agentR);
  }
};

class CircularObstacle : public Obstacle {
public:
  double radius;

  CircularObstacle(double x, double y, double radius) : Obstacle(x, y), radius(radius) {}

  bool constrained(const Point &other_point, const double other_radius) override {
    return (calculateDistance(point, other_point) <= radius + other_radius);
  }
};

std::tuple<double, double, double> computeFinalState(const std::vector<Control> &controls, double start_pos,
                                                     double start_velocity = 0.0);
void validateControlPath(const ControlPath &cp, const Point &start_pos, const Velocity &start_vel,
                         const Point &goal_pos, const Velocity goal_vel, double tolerance);
void validateControl(const Control &control, double from_pos, double from_vel, double to_pos, double to_vel,
                     double tolerance);
#endif // COMMON_H
