cmake_minimum_required(VERSION 3.0.2)
project(task_5_mr)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  run.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Laser_Params.cfg
  cfg/Odom_Params.cfg
)

catkin_package(
LIBRARIES task_5_mr
CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure
)

include_directories (
    ${catkin_INCLUDE_DIRS}
    include task_5_mr
)

# -------------------------------------------------------

add_executable (tutorial_odom_v5 src/tutorial_odom_v5.cpp src/tutorial_odom.cpp)
add_dependencies(tutorial_odom_v5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (tutorial_odom_v5 ${catkin_LIBRARIES})

add_executable (tutorial_laser_v5 src/tutorial_laser_v5.cpp src/tutorial_laser.cpp )
add_dependencies(tutorial_laser_v5 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (tutorial_laser_v5 ${catkin_LIBRARIES})

add_executable (start_robot src/start_robot.cpp)
add_dependencies(start_robot  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (start_robot ${catkin_LIBRARIES})

add_executable (stop_robot src/stop_robot.cpp)
add_dependencies(stop_robot  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (stop_robot ${catkin_LIBRARIES})






