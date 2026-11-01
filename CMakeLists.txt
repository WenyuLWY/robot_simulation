cmake_minimum_required(VERSION 3.0.2)
project(robot_simulation)

set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    pcl_conversions
    pcl_ros
    std_msgs
    gazebo_msgs
    sensor_msgs
    nav_msgs
    )


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

set(COMMON_LIBRARIES ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(localization_sim src/localization_sim.cpp)
target_link_libraries(localization_sim ${COMMON_LIBRARIES})

add_executable(sensor_input src/sensor_input.cpp)
target_link_libraries(sensor_input ${COMMON_LIBRARIES})