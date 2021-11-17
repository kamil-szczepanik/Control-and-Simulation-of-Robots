cmake_minimum_required(VERSION 3.5)
project(stero_mobile_init)
set(CMAKE_CXX_STANDARD 11)
# find_package(catkin REQUIRED roscpp rospy std_msgs genmsg tf2_ros costmap_2d global_planner nav_msgs base_local_planner clear_costmap_recovery rotate_recovery)
find_package(catkin REQUIRED
    COMPONENTS
    std_msgs
    tf2_ros
    costmap_2d
    global_planner
    dwa_local_planner
    rotate_recovery
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS roscpp nodelet
#   DEPENDS elektron
)



include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(navi scripts/navi.cpp)
target_link_libraries(navi ${catkin_LIBRARIES})
target_link_libraries(navi ${catkin_LIBRARIES})


install (DIRECTORY worlds/ DESTINATION share/${CMAKE_PROJECT_NAME}/worlds 
         FILES_MATCHING PATTERN "*.world")
install (DIRECTORY launch/ DESTINATION share/${CMAKE_PROJECT_NAME}/launch 
         FILES_MATCHING PATTERN "*.launch*")
install (DIRECTORY config/ DESTINATION share/${CMAKE_PROJECT_NAME}/config 
         FILES_MATCHING PATTERN "*.yaml")
install (DIRECTORY param/ DESTINATION share/${CMAKE_PROJECT_NAME}/param 
         FILES_MATCHING PATTERN "*.yaml")
