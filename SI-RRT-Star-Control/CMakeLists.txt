cmake_minimum_required(VERSION 3.16)
project(SI-RRTStar-Control)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR}/SI-RRT-Star-Control/include)

# Boost 설정
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# yaml-cpp 설정
find_package(yaml-cpp REQUIRED)

# Homebrew 경로 추가
link_directories(/opt/homebrew/lib)

# 소스 파일 탐색
file(GLOB SOURCES "src/*.cpp")

# 실행 파일 생성
add_executable(SI-RRTStar-Control ${SOURCES})

# 라이브러리 링크
target_link_libraries(SI-RRTStar-Control PRIVATE ${Boost_LIBRARIES} yaml-cpp)