cmake_minimum_required(VERSION 3.16)
project(SI-RRTStar)

set(CMAKE_CXX_STANDARD 17)

include_directories("include")

find_package(Boost REQUIRED)

find_package(yaml-cpp REQUIRED)

# Homebrew 경로 추가
link_directories(/opt/homebrew/lib)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})

file(GLOB SOURCES "src/*.cpp")

add_executable(SI-RRTStar ${SOURCES})

target_link_libraries(SI-RRTStar ${Boost_LIBRARIES})
target_link_libraries(SI-RRTStar ${YAML_CPP_LIBRARIES})
