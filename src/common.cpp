#include "common.h"

namespace fs = std::filesystem;

void openFile(ofstream &file, const string &filename) {
    file.open(filename, ios::out);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
    }
}

void writePath(ofstream &file, const Path &path) {
    for (const auto &point_time: path) {
        const auto &point = get<0>(point_time);
        double time = get<1>(point_time);
        file << "(" << get<0>(point) << "," << get<1>(point) << "," << time << ")->";
    }
    file << endl;
}

void savePath(const Path &path, const string &filename) {
    ofstream file;
    openFile(file, filename);
    if (!file.is_open()) return;

    writePath(file, path);
    file.close();

    if (!fs::exists(filename)) {
        cerr << "Failed to write file: " << filename << endl;
    }
}

void saveSolution(const Solution &solution, const string &filename) {
    ofstream file;
    openFile(file, filename);
    if (!file.is_open()) return;

    for (size_t i = 0; i < solution.size(); ++i) {
        file << "Agent " << i << ":";
        writePath(file, solution[i]);
    }
    file.close();

    if (!fs::exists(filename)) {
        cerr << "Failed to write file: " << filename << endl;
    }
}

void saveActionSolution(const ActionSolution &action_solution, const string &filename) {
    ofstream file;
    openFile(file, filename);
    if (!file.is_open()) return;

    for (size_t i = 0; i < action_solution.size(); ++i) {
        file << "Agent " << i << ":";
        for (const auto &control: action_solution[i]) {
            auto [acceleration, acc_time, dec_time] = control;
            auto [acc_x, acc_y] = acceleration;
            auto [t1_x, t1_y] = acc_time;
            auto [t2_x, t2_y] = dec_time;

            file << "(" << acc_x << "," << acc_y << ","
                    << t1_x << "," << t1_y << ","
                    << t2_x << "," << t2_y << ")->";
        }
        file << endl;
    }
    file.close();

    if (!fs::exists(filename)) {
        cerr << "Failed to write file: " << filename << endl;
    }
}

void saveData(double cost, double makespan, double duration, const string &filename) {
    ofstream file;
    openFile(file, filename);
    if (!file.is_open()) return;

    file << cost << "," << makespan << "," << duration << endl;
    file.close();

    if (!fs::exists(filename)) {
        cerr << "Failed to write file: " << filename << endl;
    }
}

double calculateDistance(Point point1, Point point2) {
    return hypot(get<0>(point1) - get<0>(point2), get<1>(point1) - get<1>(point2));
}

void generateRandomInstance(std::vector<Point>& start_points, std::vector<Point>& goal_points,
                            int num_of_agents, int width, int height,
                            std::vector<Obstacle>& obstacles, std::vector<double>& radii) {
    std::random_device rd;
    std::mt19937 gen(rd());

    for (int i = 0; i < num_of_agents; ++i) {
        std::uniform_real_distribution<> dis_width(radii[i], width - radii[i]);
        std::uniform_real_distribution<> dis_height(radii[i], height - radii[i]);

        bool valid_start = false;
        Point start_point;

        // 유효한 시작점 생성
        while (!valid_start) {
            start_point = {dis_width(gen), dis_height(gen)};
            valid_start = true;

            // 장애물과의 충돌 확인
            for (auto& obstacle : obstacles) {
                if (obstacle.constrained(start_point, radii[i])) {
                    valid_start = false;
                    break;
                }
            }

            // 기존 시작점들과의 충돌 확인
            for (const auto& existing_start : start_points) {
                if (calculateDistance(start_point, existing_start) < radii[i] + radii[start_points.size()]) {
                    valid_start = false;
                    break;
                }
            }
        }

        start_points.push_back(start_point);

        // 유효한 목표점 생성 (시작점과 동일한 로직 적용)
        bool valid_goal = false;
        Point goal_point;

        while (!valid_goal) {
            goal_point = {dis_width(gen), dis_height(gen)};
            valid_goal = true;

            // 장애물과의 충돌 확인
            for (auto& obstacle : obstacles) {
                if (obstacle.constrained(goal_point, radii[i])) {
                    valid_goal = false;
                    break;
                }
            }

            // 기존 목표점들과의 충돌 확인
            for (const auto& existing_goal : goal_points) {
                if (calculateDistance(goal_point, existing_goal) < radii[i] + radii[goal_points.size()]) {
                    valid_goal = false;
                    break;
                }
            }

            // 시작점과 목표점이 같지 않도록 확인
            if (calculateDistance(start_point, goal_point) < radii[i] * 2) {
                valid_goal = false;
            }
        }

        goal_points.push_back(goal_point);
    }
}
