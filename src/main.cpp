#include "ConstraintTable.h"
#include "SICBS.h"
#include "SIRRT.h"
#include "SharedEnv.h"
#include "common.h"

int main(int argc, char* argv[]) {
  string mapname;
  string obs;
  string robotnum;
  string testnum;
  string algorithm;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
      mapname = argv[i + 1];
    } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
      obs = argv[i + 1];
    } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
      robotnum = argv[i + 1];
    } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
      testnum = argv[i + 1];
    } else if (strcmp(argv[i], "-a") == 0 && i + 1 < argc) {
      algorithm = argv[i + 1];
    }
  }

  string benchmark_path = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                         robotnum + "_" + testnum + ".yaml";
  string solution_path = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                        robotnum + "_" + testnum + "_solution.txt";
  string action_solution_path = "control/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                        robotnum + "_" + testnum + "_controls.txt";
  string data_path = "data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum +
                    "_" + testnum + "_data.txt";
  YAML::Node config = YAML::LoadFile(benchmark_path);

  vector<shared_ptr<Obstacle>> obstacles;
  for (size_t i = 0; i < config["obstacles"].size(); ++i) {
    if (mapname == "CircleEnv") {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto radius = config["obstacles"][i]["radius"].as<double>();
      obstacles.emplace_back(make_shared<CircularObstacle>(center[0], center[1], radius));
    } else {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto height = config["obstacles"][i]["height"].as<double>();
      auto width = config["obstacles"][i]["width"].as<double>();
      obstacles.emplace_back(make_shared<RectangularObstacle>(center[0], center[1], width, height));
    }
  }

  // Extract start and goal states from the benchmark structure
  vector<State> start_states;
  vector<State> goal_states;
  vector<Point> start_points; // For SharedEnv compatibility
  vector<Point> goal_points;  // For SharedEnv compatibility

  for (size_t i = 0; i < config["agents"].size(); ++i) {
    auto start_pos = config["agents"][i]["startState"]["position"].as<std::vector<double>>();
    auto goal_pos = config["agents"][i]["goalState"]["position"].as<std::vector<double>>();

    auto start_angle = config["agents"][i]["startState"]["angle"].as<double>();
    auto goal_angle = config["agents"][i]["goalState"]["angle"].as<double>();

    Point start_point(start_pos[0], start_pos[1]);
    Point goal_point(goal_pos[0], goal_pos[1]);
    start_states.emplace_back(start_point, start_angle);
    goal_states.emplace_back(goal_point, goal_angle);

    // For SharedEnv compatibility (extract just x, y)
    start_points.emplace_back(start_point);
    goal_points.emplace_back(goal_point);
  }

  // Get environment parameters from benchmark
  int num_of_agents = config["agentNum"].as<int>();
  double width = config["width"] ? config["width"].as<double>() : 40.0;
  double height = config["height"] ? config["height"].as<double>() : 40.0;
  double robot_radius = config["robotRadius"] ? config["robotRadius"].as<double>() : 0.5;

  // Validate that we have the correct number of agents
  if (start_states.size() != static_cast<size_t>(num_of_agents) ||
      goal_states.size() != static_cast<size_t>(num_of_agents)) {
    cerr << "Error: Number of agents mismatch. Expected: " << num_of_agents
         << ", Got start states: " << start_states.size()
         << ", Got goal states: " << goal_states.size() << endl;
    return -1;
  }

  cout << "=== Benchmark Information ===" << endl;
  cout << "Map: " << mapname << "_" << obs << endl;
  cout << "Agents: " << num_of_agents << endl;
  cout << "Environment: " << width << "x" << height << endl;
  cout << "Robot radius: " << robot_radius << endl;
  cout << "Obstacles: " << obstacles.size() << endl;
  cout << "Algorithm: " << algorithm << endl;
  cout << "============================" << endl;

  vector<double> radii;
  vector<double> max_expand_distances;
  vector<double> max_velocities;
  vector<double> thresholds;
  vector<int> iterations;
  vector<double> goal_sample_rates;

  for (int i = 0; i < num_of_agents; ++i) {
    radii.emplace_back(robot_radius);
    max_expand_distances.emplace_back(5.0);
    max_velocities.emplace_back(0.5);
    thresholds.emplace_back(0.01);
    iterations.emplace_back(1500);
    goal_sample_rates.emplace_back(10.0);
  }

  // SharedEnv는 기존 Point 구조를 사용 (내부에서는 position만 필요)
  SharedEnv env = SharedEnv(num_of_agents, static_cast<int>(width), static_cast<int>(height),
                           start_points, goal_points, radii, max_expand_distances, max_velocities,
                           iterations, goal_sample_rates, obstacles, algorithm);
  ConstraintTable constraint_table(env);
  Solution solution;
  ActionSolution action_solution;

  auto start = std::chrono::high_resolution_clock::now();
  double sum_of_costs = 0.0;
  double makespan = 0.0;

  if (algorithm == "cbs") {
    // SI-CCBS
    // SICBS sicbs(env, constraint_table);
    // solution = sicbs.run();
    // sum_of_costs = sicbs.sum_of_costs;
    // makespan = sicbs.makespan;
  } else if (algorithm == "pp") {
    // SI-CPP
    for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
      SIRRT sirrt(agent_id, env, constraint_table);
      auto [path, control_inputs] = sirrt.run();
      cout << "Agent " << agent_id << " found a solution" << endl;
      solution.emplace_back(path);
      action_solution.emplace_back(control_inputs);
      sum_of_costs += get<1>(path.back());
      makespan = max(makespan, get<1>(path.back()));
      constraint_table.path_table[agent_id] = path;
      constraint_table.control_inputs_table[agent_id] = control_inputs;
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;

  cout << "sum of cost: " << sum_of_costs << endl;
  cout << "makespan: " << makespan << endl;
  cout << "computation time: " << duration.count() << endl;

  saveSolution(solution, solution_path);
  saveActionSolution(action_solution, action_solution_path);
  saveData(sum_of_costs, makespan, duration.count(), data_path);

  return 0;
}