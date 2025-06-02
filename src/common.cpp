#include "common.h"

namespace fs = std::filesystem;

// 비템플릿 유틸리티 함수들만 남김
void openFile(ofstream &file, const string &filename) {
  file.open(filename, ios::out);
  if (!file.is_open()) {
    cerr << "Error opening file: " << filename << endl;
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