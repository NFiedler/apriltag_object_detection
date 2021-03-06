cmake_minimum_required(VERSION 2.8.3)
project(apriltag_object_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  apriltags2_ros
  visualization_msgs
  tf
  moveit_ros_planning_interface
  pr2_phantom
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/marker_to_collision_object.cpp include/${PROJECT_NAME}/marker_to_collision_object.h)
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(apriltag_object_detection src/apriltag_object_detection.cpp)
target_link_libraries(apriltag_object_detection ${catkin_LIBRARIES})
add_dependencies(apriltag_object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(moveit_connector src/moveit_connector.cpp)
target_link_libraries(moveit_connector ${catkin_LIBRARIES})
target_link_libraries(moveit_connector ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)
add_dependencies(moveit_connector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
