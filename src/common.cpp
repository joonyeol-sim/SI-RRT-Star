#include "common.h"

namespace fs = std::filesystem;

bool createDirectories(const string &filepath) {
    try {
        fs::path file_path(filepath);
        fs::path dir_path = file_path.parent_path();

        if (!dir_path.empty() && !fs::exists(dir_path)) {
            fs::create_directories(dir_path);
            cout << "Created directory: " << dir_path << endl;
        }
        return true;
    } catch (const fs::filesystem_error& e) {
        cerr << "Failed to create directories for: " << filepath << " - " << e.what() << endl;
        return false;
    }
}

void openFile(ofstream &file, const string &filename) {
    // 디렉토리가 없으면 생성
    if (!createDirectories(filename)) {
        cerr << "Failed to create required directories for: " << filename << endl;
        return;
    }

    file.open(filename, ios::out);
    if (!file.is_open()) {
        cerr << "Error opening file: " << filename << endl;
    }
}

void writePath(ofstream &file, const Path &path) {
    for (const auto &state_time: path) {
        const auto &state = get<0>(state_time);
        double time = get<1>(state_time);
        file << "(" << get<0>(state.point) << "," << get<1>(state.point) << "," << state.angle << "," <<time << ")->";
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