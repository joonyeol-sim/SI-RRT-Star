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