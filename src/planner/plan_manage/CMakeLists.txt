cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

SET (CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS traj_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


