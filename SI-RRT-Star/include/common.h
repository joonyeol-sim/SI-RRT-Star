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

class BaseControl {
protected:
  pair<double, double> control_first; // (a_acc, t_acc)
  pair<double, double> control_last;  // (a_dec, t_dec)

public:
  BaseControl(double acc_first, double t_first, double acc_last, double t_last)
      : control_first{acc_first, t_first}, control_last{acc_last, t_last} {}

  virtual void print() const {
    cout << "acc_first: " << control_first.first << ", t_first: " << control_first.second << endl;
    cout << "acc_last: " << control_last.first << ", t_last: " << control_last.second << endl;
  }

  virtual ~BaseControl() = default;
};

class BasicControl : public BaseControl {
public:
  using BaseControl::BaseControl;

  void print() const override {
    cout << "=== Basic Control ===" << endl;
    BaseControl::print();
  }
};

class ConstControl : public BaseControl {
private:
  pair<double, double> constant; // (a_const, t_const)

public:
  ConstControl(double acc_first, double t_first, double acc_const, double t_const, double acc_last, double t_last)
      : BaseControl(acc_first, t_first, acc_last, t_last), constant{acc_const, t_const} {}

  void print() const override {
    cout << "=== Const Control ===" << endl;
    BaseControl::print();
    cout << "acc_const: " << constant.first << ", t_const: " << constant.second << endl;
  }
};

using Path = std::vector<std::tuple<Point, double>>;
using Interval = std::pair<double, double>;
using Conflict = std::tuple<int, int, std::tuple<Path, Path>>;
using Constraint = std::tuple<double, Path>;
using Solution = std::vector<Path>;

void openFile(ofstream &file, const string &filename);

void writePath(ofstream &file, const Path &path);

void savePath(const Path &path, const string &filename);

void saveSolution(const Solution &solution, const string &filename);

void saveData(double cost, double makespan, double duration, const string &filename);

double calculateDistance(Point point1, Point point2);

std::optional<BasicControl> findControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max);

std::optional<BasicControl> findControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max);

std::optional<BasicControl> findControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T);

std::optional<BasicControl> findControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T);

std::optional<ConstControl> findConstControlAccDec(double x1, double x2, double v1, double v2, double v_max,
                                                   double a_max);

std::optional<ConstControl> findConstControlDecAcc(double x1, double x2, double v1, double v2, double v_min,
                                                   double a_max);

std::optional<ConstControl> findConstControlAccDec_T(double x1, double x2, double v1, double v2, double v_max,
                                                     double T);

std::optional<ConstControl> findConstControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min,
                                                     double T);

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

#endif // COMMON_H
