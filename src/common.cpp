#include "common.h"

namespace fs = std::filesystem;

void openFile(ofstream &file, const string &filename) {
  file.open(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
  }
}

void writePath(ofstream &file, const Path &path) {
  for (const auto &point_time : path) {
    const auto &point = get<0>(point_time);
    double time = get<1>(point_time);
    file << "(" << point.x << "," << point.y << "," << time << ")->";
  }
  file << endl;
}

void savePath(const Path &path, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  writePath(file, path);
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

void saveSolution(const Solution &solution, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  for (size_t i = 0; i < solution.size(); ++i) {
    file << "Agent " << i << ":";
    writePath(file, solution[i]);
  }
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

void saveData(double cost, double makespan, double duration, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  file << cost << "," << makespan << "," << duration << endl;
  file.close();

  if (!fs::exists(filename)) {
    cerr << "Failed to write file: " << filename << endl;
  }
}

double calculateDistance(Point point1, Point point2) { return hypot(point1.x - point2.x, point1.y - point2.y); }

std::optional<BasicControl> findControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max) {
  double A = a_max;
  double B = 2.0 * v1;
  double C = ((v1 * v1 - v2 * v2) / (2.0 * a_max)) + (x1 - x2);

  double D = B * B - 4.0 * A * C;
  if (D < 0.0) {
    return std::nullopt;
  }
  double sqrtD = std::sqrt(D);

  double tP1 = (-B + sqrtD) / (2.0 * A);
  double tP2 = (-B - sqrtD) / (2.0 * A);

  std::vector<double> candidates;
  if (tP1 >= 0.0)
    candidates.push_back(tP1);
  if (tP2 >= 0.0)
    candidates.push_back(tP2);
  if (candidates.empty()) {
    return std::nullopt;
  }

  std::vector<double> valid;
  for (double t : candidates) {
    double v_peak = v1 + a_max * t;
    if (v_peak <= v_max) {
      valid.push_back(t);
    }
  }

  if (valid.empty()) {
    return std::nullopt;
  }

  double best_t_switch = *std::min_element(valid.begin(), valid.end());

  double t_acc = best_t_switch;
  double a_acc = a_max;

  double v_mid = v1 + a_acc * t_acc;

  double t_dec = (v_mid - v2) / a_max;
  if (t_dec < 0.0) {
    return std::nullopt;
  }
  double a_dec = -a_max;

  BasicControl control(a_acc, t_acc, a_dec, t_dec);
  return control;
}

std::optional<BasicControl> findControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max) {
  double A = a_max;
  double B = -2.0 * v1;
  double C = ((v1 * v1 - v2 * v2) / (2.0 * a_max)) + (x2 - x1);

  double D = B * B - 4.0 * A * C;
  if (D < 0.0) {
    return std::nullopt;
  }
  double sqrtD = std::sqrt(D);

  double tP1 = (-B + sqrtD) / (2.0 * A);
  double tP2 = (-B - sqrtD) / (2.0 * A);

  std::vector<double> candidates;
  if (tP1 >= 0.0)
    candidates.push_back(tP1);
  if (tP2 >= 0.0)
    candidates.push_back(tP2);
  if (candidates.empty()) {
    return std::nullopt;
  }

  std::vector<double> valid;
  for (double t : candidates) {
    double v_peak = v1 - a_max * t;
    if (v_peak >= v_min) {
      valid.push_back(t);
    }
  }

  if (valid.empty()) {
    return std::nullopt;
  }

  double best_t_switch = *std::min_element(valid.begin(), valid.end());

  double t_dec = best_t_switch;
  double a_dec = -a_max;

  double v_mid = v1 - a_max * t_dec;

  double t_acc = (v2 - v_mid) / a_max;
  if (t_acc < 0.0) {
    return std::nullopt;
  }
  double a_acc = a_max;

  BasicControl control(a_dec, t_dec, a_acc, t_acc);
  return control;
}

std::optional<BasicControl> findControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T) {
  double A = T * T;
  double B = +1 * (2.0 * T * (v1 + v2) + 4.0 * (x1 - x2));
  double C = -(v2 - v1) * (v2 - v1);

  double D = B * B - 4.0 * A * C;
  if (D < 0.0) {
    return std::nullopt;
  }
  double sqrtD = std::sqrt(D);

  double a1 = (-B + sqrtD) / (2.0 * A);
  double a2 = (-B - sqrtD) / (2.0 * A);

  std::vector<double> candidates;
  if (a1 > 0)
    candidates.push_back(a1);
  if (a2 > 0)
    candidates.push_back(a2);
  if (candidates.empty()) {
    return std::nullopt;
  }

  auto isValid = [&](double a_candidate) -> bool {
    double t_switch = (T / 2.0) + ((v2 - v1) / (2.0 * a_candidate));
    if (t_switch < 0.0 || t_switch > T)
      return false;

    double v_peak = v1 + a_candidate * t_switch;
    if (v_peak > v_max) {
      return false;
    }
    return true;
  };

  std::vector<double> valid;
  for (auto a_cand : candidates) {
    if (isValid(a_cand)) {
      valid.push_back(a_cand);
    }
  }
  if (valid.empty()) {
    return std::nullopt;
  }

  double best_a = *std::min_element(valid.begin(), valid.end());

  double t_switch = (T / 2.0) + ((v2 - v1) / (2.0 * best_a));
  double t_acc = t_switch;
  double t_dec = T - t_switch;

  BasicControl ctrl(best_a, t_acc, -best_a, t_dec);
  return ctrl;
}

std::optional<BasicControl> findControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T) {
  double A = T * T;
  double B = -1 * (2.0 * T * (v1 + v2) + 4.0 * (x1 - x2));
  double C = -(v2 - v1) * (v2 - v1);

  double D = B * B - 4.0 * A * C;
  if (D < 0) {
    return std::nullopt;
  }
  double sqrtD = std::sqrt(D);

  double a1 = (-B + sqrtD) / (2.0 * A);
  double a2 = (-B - sqrtD) / (2.0 * A);

  std::vector<double> candidates;
  if (a1 > 0)
    candidates.push_back(a1);
  if (a2 > 0)
    candidates.push_back(a2);
  if (candidates.empty()) {
    return std::nullopt;
  }

  auto isValid = [&](double a_candidate) -> bool {
    double t_switch = (T / 2.0) - ((v2 - v1) / (2.0 * a_candidate));
    if (t_switch < 0.0 || t_switch > T)
      return false;

    double v_peak = v1 - a_candidate * t_switch;
    if (v_peak < v_min) {
      return false;
    }
    return true;
  };

  std::vector<double> valid;
  for (auto a_cand : candidates) {
    if (isValid(a_cand)) {
      valid.push_back(a_cand);
    }
  }
  if (valid.empty()) {
    return std::nullopt;
  }

  double best_a = *std::min_element(valid.begin(), valid.end());

  double t_switch = (T / 2.0) - ((v2 - v1) / (2.0 * best_a));
  double t_dec = t_switch;
  double t_acc = T - t_switch;

  BasicControl ctrl(-best_a, t_dec, best_a, t_acc);
  return ctrl;
}

// v_max: 최대 속도 (양수)
std::optional<ConstControl> findConstControlAccDec(double x1, double x2, double v1, double v2, double v_max,
                                                   double a_max) {
  double t_acc = (v_max - v1) / a_max;
  double t_dec = (v_max - v2) / a_max;

  double numerator = (v1 * v1) + (v2 * v2) - 2.0 * (v_max * v_max);
  double denominator = 2.0 * a_max * v_max;
  double t_const = ((x2 - x1) / v_max) + (numerator / denominator);

  if (t_acc < 0.0 || t_dec < 0.0 || t_const < 0.0) {
    return std::nullopt;
  }

  double a_acc = a_max;
  double a_const = 0.0;
  double a_dec = -a_max;
  ConstControl control(a_acc, t_acc, a_const, t_const, a_dec, t_dec);

  return control;
}

// v_min: 최소 속도 (음수)
std::optional<ConstControl> findConstControlDecAcc(double x1, double x2, double v1, double v2, double v_min,
                                                   double a_max) {
  double t_dec = (v1 - v_min) / a_max;
  double t_acc = (v2 - v_min) / a_max;

  double numerator = (v1 * v1) + (v2 * v2) - 2.0 * (v_min * v_min);
  double denominator = 2.0 * a_max * v_min;
  double t_const = ((x2 - x1) / v_min) - (numerator / denominator);

  if (t_dec < 0.0 || t_acc < 0.0 || t_const < 0.0) {
    return std::nullopt;
  }

  double a_dec = -a_max;
  double a_const = 0.0;
  double a_acc = a_max;

  ConstControl control(a_dec, t_dec, a_const, t_const, a_acc, t_acc);
  return control;
}

std::optional<ConstControl> findConstControlAccDec_T(double x1, double x2, double v1, double v2, double v_max,
                                                     double T) {
  double numerator = (v_max * v_max) - v_max * (v1 + v2) + 0.5 * (v1 * v1 + v2 * v2);
  double denominator = (T * v_max) - (x2 - x1);

  if (std::fabs(denominator) < 1e-12) {
    return std::nullopt;
  }
  double a = numerator / denominator;

  if (a <= 0.0) {
    return std::nullopt;
  }

  // Calculate like findConstControlAccDec
  double t_acc = (v_max - v1) / a;
  double t_dec = (v_max - v2) / a;

  double numerator2 = (v1 * v1) + (v2 * v2) - 2.0 * (v_max * v_max);
  double denominator2 = 2.0 * a * v_max;
  double t_const = ((x2 - x1) / v_max) + (numerator2 / denominator2);

  if (t_acc < 0.0 || t_dec < 0.0 || t_const < 0.0) {
    return std::nullopt;
  }

  double a_acc = a;
  double a_const = 0.0;
  double a_dec = -a;
  ConstControl control(a_acc, t_acc, a_const, t_const, a_dec, t_dec);

  return control;
}

std::optional<ConstControl> findConstControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min,
                                                     double T) {
  double numerator = (v_min * v_min) - v_min * (v1 + v2) + 0.5 * (v1 * v1 + v2 * v2);
  double denominator = (x2 - x1) - (T * v_min);

  if (std::fabs(denominator) < 1e-12) {
    return std::nullopt;
  }
  double a = numerator / denominator;

  if (a <= 0.0) {
    return std::nullopt;
  }

  // Calculate like findConstControlDecAcc
  double t_dec = (v1 - v_min) / a;
  double t_acc = (v2 - v_min) / a;

  double numerator2 = (v1 * v1) + (v2 * v2) - 2.0 * (v_min * v_min);
  double denominator2 = 2.0 * a * v_min;
  double t_const = ((x2 - x1) / v_min) - (numerator2 / denominator2);

  if (t_dec < 0.0 || t_acc < 0.0 || t_const < 0.0) {
    return std::nullopt;
  }

  double a_dec = -a;
  double a_const = 0.0;
  double a_acc = a;

  ConstControl control(a_dec, t_dec, a_const, t_const, a_acc, t_acc);
  return control;
}