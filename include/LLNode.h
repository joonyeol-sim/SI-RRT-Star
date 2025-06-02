#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

template<int DIM = DEFAULT_DIM>
class LLNode {
public:
  State<DIM> state;
  shared_ptr<LLNode<DIM>> parent;
  double earliest_arrival_time;
  Interval interval;

  // 기본 생성자
  LLNode() : earliest_arrival_time(numeric_limits<double>::infinity()),
             interval(0.0, numeric_limits<double>::infinity()), parent(nullptr) {}

  // 상태와 시간 구간으로 초기화하는 생성자
  explicit LLNode(const State<DIM> &s, double lower_bound, double upper_bound)
      : state(s), earliest_arrival_time(numeric_limits<double>::infinity()),
        interval(lower_bound, upper_bound), parent(nullptr) {}

  // 상태, 도착 시간, 시간 구간으로 초기화하는 생성자
  LLNode(const State<DIM> &s, double arrival_time, double lower_bound, double upper_bound)
      : state(s), earliest_arrival_time(arrival_time),
        interval(lower_bound, upper_bound), parent(nullptr) {}

  // 복사 생성자
  LLNode(const LLNode<DIM> &other)
      : state(other.state), parent(other.parent),
        earliest_arrival_time(other.earliest_arrival_time),
        interval(other.interval) {}

  // 이동 생성자
  LLNode(LLNode<DIM> &&other) noexcept
      : state(std::move(other.state)), parent(std::move(other.parent)),
        earliest_arrival_time(other.earliest_arrival_time),
        interval(other.interval) {}

  // 복사 대입 연산자
  LLNode<DIM>& operator=(const LLNode<DIM> &other) {
    if (this != &other) {
      state = other.state;
      parent = other.parent;
      earliest_arrival_time = other.earliest_arrival_time;
      interval = other.interval;
    }
    return *this;
  }

  // 이동 대입 연산자
  LLNode<DIM>& operator=(LLNode<DIM> &&other) noexcept {
    if (this != &other) {
      state = std::move(other.state);
      parent = std::move(other.parent);
      earliest_arrival_time = other.earliest_arrival_time;
      interval = other.interval;
    }
    return *this;
  }

  // 소멸자
  ~LLNode() = default;

  // 유틸리티 함수들
  bool hasParent() const {
    return parent != nullptr;
  }

  double getIntervalLength() const {
    return interval.second - interval.first;
  }

  bool isInInterval(double time) const {
    return time >= interval.first && time <= interval.second;
  }

  // 디버깅을 위한 출력 함수
  void print() const {
    cout << "LLNode - State: (";
    for (int i = 0; i < DIM; ++i) {
      cout << state[i];
      if (i < DIM - 1) cout << ", ";
    }
    cout << "), Arrival: " << earliest_arrival_time
         << ", Interval: [" << interval.first << ", " << interval.second << "]" << endl;
  }
};

// 2D 특화 타입 별명 (하위 호환성)
using LLNode2D = LLNode<2>;

#endif // LLNODE_H