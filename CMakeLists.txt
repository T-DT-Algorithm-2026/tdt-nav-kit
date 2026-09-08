cmake_minimum_required(VERSION 3.1)
project(play)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # #使用release o2
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)
add_executable(play
    main.cpp
    ./src/YAstar/yastar.cpp
    ./src/YAstar/kinodynamicAstar.cpp
    ./src/MinimumSnapOsqp/minimumSnap.cpp
    ./src/MinimumSnapOsqp/sfcSquare.cpp
)

target_link_libraries(play
    ${OpenCV_LIBS}
    ${EIGEN3_LIBRARIES}
    OsqpEigen::OsqpEigen
)