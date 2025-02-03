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

void saveControlSolution(const std::vector<ControlPath> &control_solution, const string &filename) {
  ofstream file;
  openFile(file, filename);
  if (!file.is_open())
    return;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "control_solution" << YAML::Value << YAML::BeginSeq;

  // 각 ControlPath는 (x_controls, y_controls) 튜플임
  for (const auto &control_path : control_solution) {
    const auto &x_controls = std::get<0>(control_path);
    const auto &y_controls = std::get<1>(control_path);

    out << YAML::BeginMap;

    // x_controls 기록
    out << YAML::Key << "x_controls" << YAML::Value << YAML::BeginSeq;
    for (const auto &control : x_controls) {
      out << YAML::BeginMap;

      out << YAML::Key << "control_first" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_first.first << YAML::Key << "t" << YAML::Value
          << control.control_first.second << YAML::EndMap;

      out << YAML::Key << "control_const" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_const.first << YAML::Key << "t" << YAML::Value
          << control.control_const.second << YAML::EndMap;

      out << YAML::Key << "control_last" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_last.first << YAML::Key << "t" << YAML::Value << control.control_last.second
          << YAML::EndMap;

      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // y_controls 기록
    out << YAML::Key << "y_controls" << YAML::Value << YAML::BeginSeq;
    for (const auto &control : y_controls) {
      out << YAML::BeginMap;

      out << YAML::Key << "control_first" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_first.first << YAML::Key << "t" << YAML::Value
          << control.control_first.second << YAML::EndMap;

      out << YAML::Key << "control_const" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_const.first << YAML::Key << "t" << YAML::Value
          << control.control_const.second << YAML::EndMap;

      out << YAML::Key << "control_last" << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "a"
          << YAML::Value << control.control_last.first << YAML::Key << "t" << YAML::Value << control.control_last.second
          << YAML::EndMap;

      out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;
  }

  out << YAML::EndSeq;
  out << YAML::EndMap;

  file << out.c_str();
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

std::optional<Control> findControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max) {
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

  Control control(a_acc, t_acc, a_dec, t_dec);
  return control;
}

std::optional<Control> findControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max) {
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

  Control control(a_dec, t_dec, a_acc, t_acc);
  return control;
}

std::optional<Control> findControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T) {
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

  Control ctrl(best_a, t_acc, -best_a, t_dec);
  return ctrl;
}

std::optional<Control> findControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T) {
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

  Control ctrl(-best_a, t_dec, best_a, t_acc);
  return ctrl;
}

// v_max: 최대 속도 (양수)
std::optional<Control> findConstControlAccDec(double x1, double x2, double v1, double v2, double v_max, double a_max) {
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
  Control control(a_acc, t_acc, a_dec, t_dec, a_const, t_const);

  return control;
}

// v_min: 최소 속도 (음수)
std::optional<Control> findConstControlDecAcc(double x1, double x2, double v1, double v2, double v_min, double a_max) {
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

  Control control(a_dec, t_dec, a_acc, t_acc, a_const, t_const);
  return control;
}

std::optional<Control> findConstControlAccDec_T(double x1, double x2, double v1, double v2, double v_max, double T) {
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
  Control control(a_acc, t_acc, a_dec, t_dec, a_const, t_const);

  return control;
}

std::optional<Control> findConstControlDecAcc_T(double x1, double x2, double v1, double v2, double v_min, double T) {
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

  Control control(a_dec, t_dec, a_acc, t_acc, a_const, t_const);
  return control;
}

std::optional<double> calculateCostToGo(const Point &start, const Point &goal, const Velocity &v_start,
                                        const Velocity &v_goal, double v_max, double a_max) {

  std::vector<double> time_candidates_x;
  std::vector<double> time_candidates_y;

  if (auto control_x1 = findConstControlAccDec(start.x, goal.x, v_start.x, v_goal.x, v_max, a_max)) {
    time_candidates_x.push_back(control_x1->control_first.second + control_x1->control_const.second +
                                control_x1->control_last.second);
  }
  if (auto control_x2 = findConstControlDecAcc(start.x, goal.x, v_start.x, v_goal.x, -v_max, a_max)) {
    time_candidates_x.push_back(control_x2->control_first.second + control_x2->control_const.second +
                                control_x2->control_last.second);
  }
  if (auto control_x3 = findControlAccDec(start.x, goal.x, v_start.x, v_goal.x, v_max, a_max)) {
    time_candidates_x.push_back(control_x3->control_first.second + control_x3->control_last.second);
  }
  if (auto control_x4 = findControlDecAcc(start.x, goal.x, v_start.x, v_goal.x, -v_max, a_max)) {
    time_candidates_x.push_back(control_x4->control_first.second + control_x4->control_last.second);
  }

  if (time_candidates_x.empty())
    return std::nullopt;
  double T_x = *std::min_element(time_candidates_x.begin(), time_candidates_x.end());

  if (auto control_y1 = findConstControlAccDec(start.y, goal.y, v_start.y, v_goal.y, v_max, a_max)) {
    time_candidates_y.push_back(control_y1->control_first.second + control_y1->control_const.second +
                                control_y1->control_last.second);
  }
  if (auto control_y2 = findConstControlDecAcc(start.y, goal.y, v_start.y, v_goal.y, -v_max, a_max)) {
    time_candidates_y.push_back(control_y2->control_first.second + control_y2->control_const.second +
                                control_y2->control_last.second);
  }
  if (auto control_y3 = findControlAccDec(start.y, goal.y, v_start.y, v_goal.y, v_max, a_max)) {
    time_candidates_y.push_back(control_y3->control_first.second + control_y3->control_last.second);
  }
  if (auto control_y4 = findControlDecAcc(start.y, goal.y, v_start.y, v_goal.y, -v_max, a_max)) {
    time_candidates_y.push_back(control_y4->control_first.second + control_y4->control_last.second);
  }

  if (time_candidates_y.empty())
    return std::nullopt;
  double T_y = *std::min_element(time_candidates_y.begin(), time_candidates_y.end());

  return std::max(T_x, T_y);
}

std::optional<std::tuple<double, std::unique_ptr<Control>, std::unique_ptr<Control>>>
calculateCostToGoWithControls(const Point &start, const Point &goal, const Velocity &v_start, const Velocity &v_goal,
                              double v_max, double a_max) {
  std::vector<std::tuple<double, std::unique_ptr<Control>>> x_controls;
  std::vector<std::tuple<double, std::unique_ptr<Control>>> y_controls;

  // Compute all possible controls for x direction
  // Type 1: Constant velocity phase with acceleration then deceleration (ACC-CONST-DEC)
  if (auto control_acc_const_dec = findConstControlAccDec(start.x, goal.x, v_start.x, v_goal.x, v_max, a_max)) {
    x_controls.emplace_back(control_acc_const_dec->control_first.second + control_acc_const_dec->control_const.second +
                                control_acc_const_dec->control_last.second,
                            std::make_unique<Control>(*control_acc_const_dec));
  }

  // Type 2: Constant velocity phase with deceleration then acceleration (DEC-CONST-ACC)
  if (auto control_dec_const_acc = findConstControlDecAcc(start.x, goal.x, v_start.x, v_goal.x, -v_max, a_max)) {
    x_controls.emplace_back(control_dec_const_acc->control_first.second + control_dec_const_acc->control_const.second +
                                control_dec_const_acc->control_last.second,
                            std::make_unique<Control>(*control_dec_const_acc));
  }

  // Type 3: Direct acceleration then deceleration (ACC-DEC)
  if (auto control_acc_dec = findControlAccDec(start.x, goal.x, v_start.x, v_goal.x, v_max, a_max)) {
    x_controls.emplace_back(control_acc_dec->control_first.second + control_acc_dec->control_last.second,
                            std::make_unique<Control>(*control_acc_dec));
  }

  // Type 4: Direct deceleration then acceleration (DEC-ACC)
  if (auto control_dec_acc = findControlDecAcc(start.x, goal.x, v_start.x, v_goal.x, -v_max, a_max)) {
    x_controls.emplace_back(control_dec_acc->control_first.second + control_dec_acc->control_last.second,
                            std::make_unique<Control>(*control_dec_acc));
  }

  // Compute all possible controls for y direction
  // Type 1: ACC-CONST-DEC
  if (auto control_acc_const_dec = findConstControlAccDec(start.y, goal.y, v_start.y, v_goal.y, v_max, a_max)) {
    y_controls.emplace_back(control_acc_const_dec->control_first.second + control_acc_const_dec->control_const.second +
                                control_acc_const_dec->control_last.second,
                            std::make_unique<Control>(*control_acc_const_dec));
  }

  // Type 2: DEC-CONST-ACC
  if (auto control_dec_const_acc = findConstControlDecAcc(start.y, goal.y, v_start.y, v_goal.y, -v_max, a_max)) {
    y_controls.emplace_back(control_dec_const_acc->control_first.second + control_dec_const_acc->control_const.second +
                                control_dec_const_acc->control_last.second,
                            std::make_unique<Control>(*control_dec_const_acc));
  }

  // Type 3: ACC-DEC
  if (auto control_acc_dec = findControlAccDec(start.y, goal.y, v_start.y, v_goal.y, v_max, a_max)) {
    y_controls.emplace_back(control_acc_dec->control_first.second + control_acc_dec->control_last.second,
                            std::make_unique<Control>(*control_acc_dec));
  }

  // Type 4: DEC-ACC
  if (auto control_dec_acc = findControlDecAcc(start.y, goal.y, v_start.y, v_goal.y, -v_max, a_max)) {
    y_controls.emplace_back(control_dec_acc->control_first.second + control_dec_acc->control_last.second,
                            std::make_unique<Control>(*control_dec_acc));
  }

  if (x_controls.empty() || y_controls.empty()) {
    return std::nullopt;
  }

  // Find minimum time for each axis
  auto best_x = std::min_element(x_controls.begin(), x_controls.end(),
                                 [](const auto &a, const auto &b) { return std::get<0>(a) < std::get<0>(b); });
  auto best_y = std::min_element(y_controls.begin(), y_controls.end(),
                                 [](const auto &a, const auto &b) { return std::get<0>(a) < std::get<0>(b); });

  double T_x = std::get<0>(*best_x);
  double T_y = std::get<0>(*best_y);
  double final_T = std::max(T_x, T_y);

  std::unique_ptr<Control> final_x_control;
  std::unique_ptr<Control> final_y_control;

  // Helper function to get maximum acceleration from a control
  auto get_max_acceleration = [](const Control *control) -> double {
    if (const auto *const_control = dynamic_cast<const Control *>(control)) {
      return std::max({std::abs(const_control->control_first.first), std::abs(const_control->control_const.first),
                       std::abs(const_control->control_last.first)});
    } else if (const auto *basic_control = dynamic_cast<const Control *>(control)) {
      return std::max(std::abs(basic_control->control_first.first), std::abs(basic_control->control_last.first));
    }
    return std::numeric_limits<double>::infinity();
  };

  // For the axis with shorter time, collect all possible controls with final_T
  if (T_x < final_T) {
    std::vector<std::unique_ptr<Control>> x_candidates;

    // Type 1: ACC-CONST-DEC with specified time
    if (auto control = findConstControlAccDec_T(start.x, goal.x, v_start.x, v_goal.x, v_max, final_T)) {
      x_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 2: DEC-CONST-ACC with specified time
    if (auto control = findConstControlDecAcc_T(start.x, goal.x, v_start.x, v_goal.x, -v_max, final_T)) {
      x_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 3: ACC-DEC with specified time
    if (auto control = findControlAccDec_T(start.x, goal.x, v_start.x, v_goal.x, v_max, final_T)) {
      x_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 4: DEC-ACC with specified time
    if (auto control = findControlDecAcc_T(start.x, goal.x, v_start.x, v_goal.x, -v_max, final_T)) {
      x_candidates.push_back(std::make_unique<Control>(*control));
    }

    if (x_candidates.empty()) {
      return std::nullopt; // Failed to find any synchronized controls
    }

    // Find control with minimum acceleration
    auto min_acc_it = std::min_element(x_candidates.begin(), x_candidates.end(),
                                       [&get_max_acceleration](const auto &a, const auto &b) {
                                         return get_max_acceleration(a.get()) < get_max_acceleration(b.get());
                                       });

    final_x_control = std::move(*min_acc_it);
    final_y_control = std::move(std::get<1>(*best_y));

  } else if (T_y < final_T) {
    std::vector<std::unique_ptr<Control>> y_candidates;

    // Type 1: ACC-CONST-DEC with specified time
    if (auto control = findConstControlAccDec_T(start.y, goal.y, v_start.y, v_goal.y, v_max, final_T)) {
      y_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 2: DEC-CONST-ACC with specified time
    if (auto control = findConstControlDecAcc_T(start.y, goal.y, v_start.y, v_goal.y, -v_max, final_T)) {
      y_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 3: ACC-DEC with specified time
    if (auto control = findControlAccDec_T(start.y, goal.y, v_start.y, v_goal.y, v_max, final_T)) {
      y_candidates.push_back(std::make_unique<Control>(*control));
    }
    // Type 4: DEC-ACC with specified time
    if (auto control = findControlDecAcc_T(start.y, goal.y, v_start.y, v_goal.y, -v_max, final_T)) {
      y_candidates.push_back(std::make_unique<Control>(*control));
    }

    if (y_candidates.empty()) {
      return std::nullopt; // Failed to find any synchronized controls
    }

    // Find control with minimum acceleration
    auto min_acc_it = std::min_element(y_candidates.begin(), y_candidates.end(),
                                       [&get_max_acceleration](const auto &a, const auto &b) {
                                         return get_max_acceleration(a.get()) < get_max_acceleration(b.get());
                                       });

    final_y_control = std::move(*min_acc_it);
    final_x_control = std::move(std::get<1>(*best_x));

  } else {
    // Times are already equal
    final_x_control = std::move(std::get<1>(*best_x));
    final_y_control = std::move(std::get<1>(*best_y));
  }

  // Validate final states before returning
  auto compute_final_state = [](double x0, double v0, const Control *control) -> std::tuple<double, double, double> {
    double x = x0;  // position
    double v = v0;  // velocity
    double t = 0.0; // time

    if (const auto *const_control = dynamic_cast<const Control *>(control)) {
      // First phase
      double t1 = const_control->control_first.second;
      double a1 = const_control->control_first.first;
      x += v0 * t1 + 0.5 * a1 * t1 * t1;
      v += a1 * t1;
      t += t1;

      // Constant velocity phase
      t += const_control->control_const.second;
      x += v * const_control->control_const.second;

      // Final phase
      double t2 = const_control->control_last.second;
      double a2 = const_control->control_last.first;
      x += v * t2 + 0.5 * a2 * t2 * t2;
      v += a2 * t2;
      t += t2;

    } else if (const auto *basic_control = dynamic_cast<const Control *>(control)) {
      // First phase
      double t1 = basic_control->control_first.second;
      double a1 = basic_control->control_first.first;
      x += v0 * t1 + 0.5 * a1 * t1 * t1;
      v += a1 * t1;
      t += t1;

      // Second phase
      double t2 = basic_control->control_last.second;
      double a2 = basic_control->control_last.first;
      x += v * t2 + 0.5 * a2 * t2 * t2;
      v += a2 * t2;
      t += t2;
    }

    return {x, v, t};
  };

  // Check x-axis
  auto [x_pos, x_vel, x_time] = compute_final_state(start.x, v_start.x, final_x_control.get());
  if (std::abs(x_pos - goal.x) > 1e-6 || std::abs(x_vel - v_goal.x) > 1e-6 || std::abs(x_time - final_T) > 1e-6) {
    std::cerr << "X-axis validation failed:" << std::endl
              << "Position error: " << std::abs(x_pos - goal.x) << std::endl
              << "Velocity error: " << std::abs(x_vel - v_goal.x) << std::endl
              << "Time error: " << std::abs(x_time - final_T) << std::endl;
    return std::nullopt;
  }

  // Check y-axis
  auto [y_pos, y_vel, y_time] = compute_final_state(start.y, v_start.y, final_y_control.get());
  if (std::abs(y_pos - goal.y) > 1e-6 || std::abs(y_vel - v_goal.y) > 1e-6 || std::abs(y_time - final_T) > 1e-6) {
    std::cerr << "Y-axis validation failed:" << std::endl
              << "Position error: " << std::abs(y_pos - goal.y) << std::endl
              << "Velocity error: " << std::abs(y_vel - v_goal.y) << std::endl
              << "Time error: " << std::abs(y_time - final_T) << std::endl;
    return std::nullopt;
  }

  return std::make_tuple(final_T, std::move(final_x_control), std::move(final_y_control));
}

/// @brief 단일 Control 객체를 적용하여, 주어진 축에서 시작 스테이트 (start_pos, start_velocity)로부터
///        최종 상태(위치, 속도, 총 소요시간)를 계산한다.
std::tuple<double, double, double> computeFinalState(const Control &control, double start_pos, double start_velocity) {
  double pos = start_pos;
  double v = start_velocity;
  double t_total = 0.0;

  // Phase 1: control_first
  double t1 = control.control_first.second;
  double a1 = control.control_first.first;
  pos += v * t1 + 0.5 * a1 * t1 * t1;
  v += a1 * t1;
  t_total += t1;

  // Phase 2: control_const (가속도가 0인 구간)
  double t_const = control.control_const.second;
  pos += v * t_const;
  t_total += t_const;

  // Phase 3: control_last
  double t2 = control.control_last.second;
  double a2 = control.control_last.first;
  pos += v * t2 + 0.5 * a2 * t2 * t2;
  v += a2 * t2;
  t_total += t2;

  return std::make_tuple(pos, v, t_total);
}

/// @brief 단일 Control 객체를 시작 스테이트 (from_pos, from_vel)에서 적용했을 때,
///        도착 스테이트 (to_pos, to_vel)로 정확히 도달하는지(허용 오차 tolerance 이하)를 검증한다.
///        만약 검증에 실패하면 예외(std::runtime_error)를 throw한다.
///        최종 계산된 위치, 속도, 소요시간은 출력 인자로 전달된다.
void validateControl(const Control &control, double from_pos, double from_vel, double to_pos, double to_vel,
                     double tolerance) {
  double final_pos, final_vel, t_total;
  std::tie(final_pos, final_vel, t_total) = computeFinalState(control, from_pos, from_vel);
  if (!(std::fabs(final_pos - to_pos) < tolerance && std::fabs(final_vel - to_vel) < tolerance)) {
    std::string error_msg = "Control validation failed:\n";
    error_msg += "  Computed final position = " + std::to_string(final_pos) +
                 ", goal position = " + std::to_string(to_pos) + "\n";
    error_msg += "  Computed final velocity = " + std::to_string(final_vel) +
                 ", goal velocity = " + std::to_string(to_vel) + "\n";
    error_msg += "  Total time = " + std::to_string(t_total);
    throw std::runtime_error(error_msg);
  }
}

/// @brief 주어진 ControlPath (x, y 각각의 vector<Control>)에 대해,
///        시작 스테이트 (start)에서 도착 스테이트 (goal)로의 전이가 올바르게 이루어졌는지 검증한다.
///        각 축에 대해 벡터 내의 Control 객체들을 순차적으로 적용하여 최종 상태를 계산하고,
///        그 결과가 목표 상태와 허용 오차 tolerance 이하이면 올바른 것으로 판단한다.
void validateControlPath(const ControlPath &cp, const Point &start_pos, const Velocity &start_vel,
                         const Point &goal_pos, const Velocity goal_vel, double tolerance) {
  // x축에 대해 누적 적용
  double current_x = start_pos.x;
  double current_vx = start_vel.x;
  double total_time_x = 0.0;
  for (const auto &ctrl : std::get<0>(cp)) {
    double seg_final_x, seg_final_vx, seg_t;
    std::tie(seg_final_x, seg_final_vx, seg_t) = computeFinalState(ctrl, current_x, current_vx);
    current_x = seg_final_x;
    current_vx = seg_final_vx;
    total_time_x += seg_t;
  }
  if (!(std::fabs(current_x - goal_pos.x) < tolerance && std::fabs(current_vx - goal_vel.x) < tolerance)) {
    std::string error_msg = "Control path validation failed for x-axis:\n";
    error_msg +=
        "  Computed final x = " + std::to_string(current_x) + ", goal x = " + std::to_string(goal_pos.x) + "\n";
    error_msg +=
        "  Computed final vx = " + std::to_string(current_vx) + ", goal vx = " + std::to_string(goal_vel.x) + "\n";
    error_msg += "  Total time = " + std::to_string(total_time_x);
    throw std::runtime_error(error_msg);
  }

  // y축에 대해 누적 적용
  double current_y = start_pos.y;
  double current_vy = start_vel.y;
  double total_time_y = 0.0;
  for (const auto &ctrl : std::get<1>(cp)) {
    double seg_final_y, seg_final_vy, seg_t;
    std::tie(seg_final_y, seg_final_vy, seg_t) = computeFinalState(ctrl, current_y, current_vy);
    current_y = seg_final_y;
    current_vy = seg_final_vy;
    total_time_y += seg_t;
  }
  if (!(std::fabs(current_y - goal_pos.y) < tolerance && std::fabs(current_vy - goal_vel.y) < tolerance)) {
    std::string error_msg = "Control path validation failed for y-axis:\n";
    error_msg +=
        "  Computed final y = " + std::to_string(current_y) + ", goal y = " + std::to_string(goal_pos.y) + "\n";
    error_msg +=
        "  Computed final vy = " + std::to_string(current_vy) + ", goal vy = " + std::to_string(goal_vel.y) + "\n";
    error_msg += "  Total time = " + std::to_string(total_time_y);
    throw std::runtime_error(error_msg);
  }
}