cmake_minimum_required(VERSION 3.16)
project(SI-RRTStar)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/SI-RRT-Star/include)

# Boost 설정
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# yaml-cpp 설정
find_package(yaml-cpp REQUIRED)

# Eigen3 설정 (Homebrew에서 설치된 Eigen 찾기)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

# Homebrew 경로 추가
link_directories(/opt/homebrew/lib)

# 소스 파일 탐색
file(GLOB SOURCES "src/*.cpp")

# 실행 파일 생성
add_executable(SI-RRTStar ${SOURCES})

# 라이브러리 링크
target_link_libraries(SI-RRTStar PRIVATE ${Boost_LIBRARIES} yaml-cpp)