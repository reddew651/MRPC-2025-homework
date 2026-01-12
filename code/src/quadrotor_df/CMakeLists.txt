cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_df)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  eigen_conversions
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES quadrotor_df
#  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(df_node src/df_node.cpp)
target_link_libraries(df_node
  ${catkin_LIBRARIES}
)
