project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall -DHAVE_CSTDDEF -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

include_directories(
        "src/Eigen-3.3"
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/include)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/lib)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(path_planning
        "src/behavior_planner.cpp"
        "src/behavior_planner.h"
        "src/helper.cpp"
        "src/helper.hpp"
        "src/highway.cpp"
        "src/highway.hpp"
        "src/json.hpp"
        "src/main.cpp"
        "src/settings.h"
        "src/spline.h"
        "src/trajectory_planner.cpp"
        "src/trajectory_planner.h"
        "src/vehicle.cpp"
        "src/vehicle.h"
        )

target_link_libraries(path_planning z ssl uv uWS)