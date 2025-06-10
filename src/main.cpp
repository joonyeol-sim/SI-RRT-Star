#include "ConstraintTable.h"
#include "SICBS.h"
#include "SIRRT.h"
#include "SharedEnv.h"
#include "common.h"

// 차원 설정 (PROBLEM_DIM을 변경하여 2D, 3D 등 지원)
constexpr int PROBLEM_DIM = 3;
using ProblemState = State<PROBLEM_DIM>;
using ProblemSharedEnv = SharedEnv<PROBLEM_DIM>;
using ProblemConstraintTable = ConstraintTable<PROBLEM_DIM>;
using ProblemSIRRT = SIRRT<PROBLEM_DIM>;
using ProblemSICBS = SICBS<PROBLEM_DIM>;
using ProblemSolution = Solution<PROBLEM_DIM>;
using ProblemObstacle = Obstacle<PROBLEM_DIM>;

int main(int argc, char *argv[]) {
  string mapname;
  string obs;
  string robotnum;
  string testnum;
  string algorithm;
  bool use_random = false; // 랜덤 인스턴스 사용 여부

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
    } else if (strcmp(argv[i], "--random") == 0) {
      use_random = true;
    }
  }

  string benchmarkPath = "benchmark/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                         robotnum + "_" + testnum + ".yaml";
  string solutionPath = "solution/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" +
                        robotnum + "_" + testnum + "_solution.txt";
  string dataPath = "data/" + mapname + "_" + obs + "/agents" + robotnum + "/" + mapname + "_" + obs + "_" + robotnum +
                    "_" + testnum + "_data.txt";

  YAML::Node config = YAML::LoadFile(benchmarkPath);

  // 장애물 생성 (n차원 구조에 맞게)
  vector<shared_ptr<ProblemObstacle>> obstacles;
  for (size_t i = 0; i < config["obstacles"].size(); ++i) {
    if (mapname == "CircleEnv" || mapname == "SphereEnv") {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto radius = config["obstacles"][i]["radius"].as<double>();
      // n차원 중심점에 맞게 생성 (2D의 경우)
      if constexpr (PROBLEM_DIM == 2) {
        obstacles.emplace_back(make_shared<HyperSphereObstacle<PROBLEM_DIM>>(radius, center[0], center[1]));
      } else if constexpr (PROBLEM_DIM == 3) {
        obstacles.emplace_back(make_shared<HyperSphereObstacle<PROBLEM_DIM>>(radius, center[0], center[1], center[2]));
      }
    } else {
      auto center = config["obstacles"][i]["center"].as<std::vector<double>>();
      auto height = config["obstacles"][i]["height"].as<double>();
      auto width = config["obstacles"][i]["width"].as<double>();
      // n차원 직육면체에 맞게 생성 (2D의 경우)
      if constexpr (PROBLEM_DIM == 2) {
        obstacles.emplace_back(make_shared<HyperRectangleObstacle<PROBLEM_DIM>>(center[0], center[1], width, height));
      } else if constexpr (PROBLEM_DIM == 3) {
        auto depth = config["obstacles"][i]["depth"].as<double>(1.0); // 기본값 1.0
        obstacles.emplace_back(make_shared<HyperRectangleObstacle<PROBLEM_DIM>>(center[0], center[1], center[2], width, height, depth));
      }
    }
  }

  // 환경 설정
  int num_of_agents = config["agentNum"].as<int>();
  int width = config["width"].as<int>(40.0);
  int height = config["height"].as<int>(40.0);
  int depth = config["depth"].as<int>(20.0);

  vector<double> radii;
  vector<double> max_expand_distances;
  vector<double> max_velocities;
  vector<double> thresholds;
  vector<int> iterations;
  vector<double> goal_sample_rates;

  for (int i = 0; i < num_of_agents; ++i) {
    radii.emplace_back(1.0);
    max_expand_distances.emplace_back(5.0);
    max_velocities.emplace_back(0.5);
    thresholds.emplace_back(0.01);
    iterations.emplace_back(1500);
    goal_sample_rates.emplace_back(10.0);
  }

  // 시작점과 목표점 생성
  vector<ProblemState> start_states;
  vector<ProblemState> goal_states;

  if (!use_random) {
    // YAML 파일에서 읽어오기 (기존 방식)
    start_states.reserve(config["startPoints"].size());
    goal_states.reserve(config["goalPoints"].size());
    for (size_t i = 0; i < config["startPoints"].size(); ++i) {
      auto start = config["startPoints"][i].as<std::vector<double>>();
      auto goal = config["goalPoints"][i].as<std::vector<double>>();

      // n차원 좌표에 맞게 생성
      if constexpr (PROBLEM_DIM == 2) {
        start_states.emplace_back(start[0], start[1]);
        goal_states.emplace_back(goal[0], goal[1]);
      } else if constexpr (PROBLEM_DIM == 3) {
        start_states.emplace_back(start[0], start[1], start.size() > 2 ? start[2] : 0.0);
        goal_states.emplace_back(goal[0], goal[1], goal.size() > 2 ? goal[2] : 0.0);
      } else {
        // 일반적인 n차원 처리
        std::array<double, PROBLEM_DIM> start_coords{};
        std::array<double, PROBLEM_DIM> goal_coords{};
        for (int dim = 0; dim < PROBLEM_DIM; ++dim) {
          start_coords[dim] = dim < start.size() ? start[dim] : 0.0;
          goal_coords[dim] = dim < goal.size() ? goal[dim] : 0.0;
        }
        start_states.emplace_back(start_coords);
        goal_states.emplace_back(goal_coords);
      }
    }
  }

  // SharedEnv 생성 (템플릿 버전 사용, 2D 호환성 유지)
  ProblemSharedEnv env(num_of_agents, width, height, depth, start_states, goal_states, radii, max_expand_distances,
                       max_velocities, iterations, goal_sample_rates, obstacles, algorithm);

  // 랜덤 인스턴스 생성 (use_random이 true인 경우)
  if (use_random) {
    cout << "Generating random instance..." << endl;
    env.generateRandomInstance();
    cout << "Random instance generated successfully!" << endl;

    // 생성된 시작점과 목표점 출력 (디버깅용)
    cout << "Start points:" << endl;
    for (int i = 0; i < num_of_agents; ++i) {
      cout << "Agent " << i << ": ";
      for (int dim = 0; dim < PROBLEM_DIM; ++dim) {
        cout << env.start_states[i][dim] << " ";
      }
      cout << endl;
    }

    cout << "Goal points:" << endl;
    for (int i = 0; i < num_of_agents; ++i) {
      cout << "Agent " << i << ": ";
      for (int dim = 0; dim < PROBLEM_DIM; ++dim) {
        cout << env.goal_states[i][dim] << " ";
      }
      cout << endl;
    }
  }

  ProblemConstraintTable constraint_table(env);
  ProblemSolution solution;
  auto start = std::chrono::high_resolution_clock::now();
  double sum_of_costs = 0.0;
  double makespan = 0.0;

  if (algorithm == "cbs") {
    // SI-CCBS
    ProblemSICBS sicbs(env, constraint_table);
    solution = sicbs.run();
    sum_of_costs = sicbs.sum_of_costs;
    makespan = sicbs.makespan;
  } else if (algorithm == "pp") {
    // SI-CPP
    for (int agent_id = 0; agent_id < num_of_agents; ++agent_id) {
      ProblemSIRRT sirrt(agent_id, env, constraint_table);
      auto path = sirrt.run();
      cout << "Agent " << agent_id << " found a solution" << endl;
      solution.emplace_back(path);
      sum_of_costs += get<1>(path.back());
      makespan = max(makespan, get<1>(path.back()));
      constraint_table.path_table[agent_id] = path;
    }
  }

  auto stop = std::chrono::high_resolution_clock::now();
  chrono::duration<double, std::ratio<1>> duration = stop - start;

  if (constraint_table.checkConflicts(solution)) {
    cout << "Conflict exists" << endl;
  }

  cout << "sum of cost: " << sum_of_costs << endl;
  cout << "makespan: " << makespan << endl;
  cout << "computation time: " << duration.count() << endl;

  // 템플릿 버전의 save 함수 사용
  saveSolution<PROBLEM_DIM>(solution, solutionPath);
  saveData(sum_of_costs, makespan, duration.count(), dataPath);

  return 0;
}