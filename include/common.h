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
#include <array>

using namespace std;

// 기본 차원 설정
constexpr int DEFAULT_DIM = 2;

template<int DIM = DEFAULT_DIM>
class Velocity {
public:
  std::array<double, DIM> coords;

  explicit Velocity() {
    coords.fill(0.0);
  }

  explicit Velocity(const std::array<double, DIM>& values) : coords(values) {}

  // 가변 인자 생성자
  template<typename... Args>
  explicit Velocity(Args... args) {
    static_assert(sizeof...(args) == DIM, "Number of arguments must match dimension");
    coords = {static_cast<double>(args)...};
  }

  double& operator[](int index) { return coords[index]; }
  const double& operator[](int index) const { return coords[index]; }

  bool operator==(const Velocity<DIM> &other) const {
    return coords == other.coords;
  }

  bool operator!=(const Velocity<DIM> &other) const {
    return !(*this == other);
  }

  Velocity<DIM> operator+(const Velocity<DIM> &other) const {
    Velocity<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] + other.coords[i];
    }
    return result;
  }

  Velocity<DIM> operator-(const Velocity<DIM> &other) const {
    Velocity<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] - other.coords[i];
    }
    return result;
  }

  template<typename T>
  Velocity<DIM> operator*(const T &scalar) const {
    Velocity<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] * static_cast<double>(scalar);
    }
    return result;
  }

  template<typename T>
  double dot(const T &other) const {
    double result = 0.0;
    for (int i = 0; i < DIM; ++i) {
      result += coords[i] * other.coords[i];
    }
    return result;
  }

  double length() const {
    double sum = 0.0;
    for (int i = 0; i < DIM; ++i) {
      sum += coords[i] * coords[i];
    }
    return std::sqrt(sum);
  }

  Velocity<DIM> normalize() const {
    double len = length();
    if (len == 0.0) return *this;
    return *this * (1.0 / len);
  }
};

template<int DIM = DEFAULT_DIM>
class State {
public:
  std::array<double, DIM> coords;

  explicit State() {
    coords.fill(0.0);
  }

  explicit State(const std::array<double, DIM>& values) : coords(values) {}

  // 가변 인자 생성자
  template<typename... Args>
  explicit State(Args... args) {
    static_assert(sizeof...(args) == DIM, "Number of arguments must match dimension");
    coords = {static_cast<double>(args)...};
  }

  double& operator[](int index) { return coords[index]; }
  const double& operator[](int index) const { return coords[index]; }

  bool operator==(const State<DIM> &other) const {
    return coords == other.coords;
  }

  bool operator!=(const State<DIM> &other) const {
    return !(*this == other);
  }

  State<DIM> operator+(const State<DIM> &other) const {
    State<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] + other.coords[i];
    }
    return result;
  }

  State<DIM> operator-(const State<DIM> &other) const {
    State<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] - other.coords[i];
    }
    return result;
  }

  template<typename T>
  State<DIM> operator*(const T &scalar) const {
    State<DIM> result;
    for (int i = 0; i < DIM; ++i) {
      result.coords[i] = coords[i] * static_cast<double>(scalar);
    }
    return result;
  }

  template<typename T>
  double dot(const T &other) const {
    double result = 0.0;
    for (int i = 0; i < DIM; ++i) {
      result += coords[i] * other.coords[i];
    }
    return result;
  }

  double distance(const State<DIM> &other) const {
    double sum = 0.0;
    for (int i = 0; i < DIM; ++i) {
      double diff = coords[i] - other.coords[i];
      sum += diff * diff;
    }
    return std::sqrt(sum);
  }
};

// 타입 별명
using State2D = State<2>;
using State3D = State<3>;
using Velocity2D = Velocity<2>;
using Velocity3D = Velocity<3>;

template<int DIM = DEFAULT_DIM>
using Path = std::vector<std::tuple<State<DIM>, double>>;

template<int DIM = DEFAULT_DIM>
using Conflict = std::tuple<int, int, std::tuple<Path<DIM>, Path<DIM>>>;

template<int DIM = DEFAULT_DIM>
using Constraint = std::tuple<double, Path<DIM>>;

template<int DIM = DEFAULT_DIM>
using Solution = std::vector<Path<DIM>>;

using Interval = std::pair<double, double>;

// 유틸리티 함수들
template<int DIM = DEFAULT_DIM>
Velocity<DIM> toVelocity(const State<DIM>& from, const State<DIM>& to) {
  Velocity<DIM> result;
  for (int i = 0; i < DIM; ++i) {
    result.coords[i] = to.coords[i] - from.coords[i];
  }
  return result;
}

template<int DIM = DEFAULT_DIM>
State<DIM> toState(const Velocity<DIM>& velocity) {
  State<DIM> result;
  for (int i = 0; i < DIM; ++i) {
    result.coords[i] = velocity.coords[i];
  }
  return result;
}

template<int DIM = DEFAULT_DIM>
double calculateDistance(const State<DIM>& state1, const State<DIM>& state2) {
  return state1.distance(state2);
}

template<int DIM = DEFAULT_DIM>
struct StateHash {
  size_t operator()(const State<DIM> &state) const {
    size_t seed = 0;
    for (int i = 0; i < DIM; ++i) {
      boost::hash_combine(seed, state.coords[i]);
    }
    return seed;
  }
};

// 파일 I/O 함수 선언들
void openFile(ofstream &file, const string &filename);
void saveData(double cost, double makespan, double duration, const string &filename);

template<int DIM = DEFAULT_DIM>
void writePath(ofstream &file, const Path<DIM> &path);

template<int DIM = DEFAULT_DIM>
void savePath(const Path<DIM> &path, const string &filename);

template<int DIM = DEFAULT_DIM>
void saveSolution(const Solution<DIM> &solution, const string &filename);

// 장애물 클래스들
template<int DIM = DEFAULT_DIM>
class Obstacle {
public:
  State<DIM> center;

  template<typename... Args>
  explicit Obstacle(Args... args) : center(State<DIM>(args...)) {}

  virtual ~Obstacle() = default;

  virtual bool isColliding(const State<DIM> &state, double radius) const = 0;
};

template<int DIM = DEFAULT_DIM>
class HyperRectangleObstacle : public Obstacle<DIM> {
public:
  std::array<double, DIM> dimensions;

  template<typename... Args>
  HyperRectangleObstacle(Args... args) : Obstacle<DIM>() {
    static_assert(sizeof...(args) == DIM * 2, "Arguments must be center coordinates + dimensions");
    std::array<double, DIM * 2> all_args = {static_cast<double>(args)...};

    for (int i = 0; i < DIM; ++i) {
      this->center.coords[i] = all_args[i];
      dimensions[i] = all_args[i + DIM];
    }
  }

  bool isColliding(const State<DIM> &state, double radius) const override {
    // N차원 하이퍼직육면체와의 충돌 검사
    bool inside = true;
    double dist_sq = 0.0;

    for (int i = 0; i < DIM; ++i) {
      double min_bound = this->center.coords[i] - dimensions[i] / 2;
      double max_bound = this->center.coords[i] + dimensions[i] / 2;

      if (state.coords[i] < min_bound || state.coords[i] > max_bound) {
        inside = false;
      }

      double closest = std::max(min_bound, std::min(state.coords[i], max_bound));
      double diff = state.coords[i] - closest;
      dist_sq += diff * diff;
    }

    return inside || (dist_sq <= radius * radius);
  }
};

template<int DIM = DEFAULT_DIM>
class HyperSphereObstacle : public Obstacle<DIM> {
public:
  double radius;

  template<typename... Args>
  HyperSphereObstacle(double r, Args... args) : Obstacle<DIM>(args...), radius(r) {}

  bool isColliding(const State<DIM> &state, double agent_radius) const override {
    return (this->center.distance(state) <= radius + agent_radius);
  }
};

// 편의를 위한 2D/3D 특화 타입
using RectangleObstacle = HyperRectangleObstacle<2>;
using BoxObstacle = HyperRectangleObstacle<3>;
using CircleObstacle = HyperSphereObstacle<2>;
using SphereObstacle = HyperSphereObstacle<3>;

// 템플릿 함수 구현들
template<int DIM>
void writePath(ofstream &file, const Path<DIM> &path) {
  for (const auto &state_time : path) {
    const auto &state = get<0>(state_time);
    double time = get<1>(state_time);
    file << "(";
    for (int i = 0; i < DIM; ++i) {
      file << state.coords[i];
      if (i < DIM - 1) file << ",";
    }
    file << "," << time << ")->";
  }
  file << endl;
}

template<int DIM>
void savePath(const Path<DIM> &path, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  writePath(file, path);
  file.close();

  if (!std::filesystem::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

template<int DIM>
void saveSolution(const Solution<DIM> &solution, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  for (size_t i = 0; i < solution.size(); ++i) {
    file << "Agent " << i << ": ";
    writePath(file, solution[i]);
  }
  file.close();

  if (!std::filesystem::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

#endif // COMMON_H