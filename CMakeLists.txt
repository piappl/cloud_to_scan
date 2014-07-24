cmake_minimum_required(VERSION 2.8.3)
project(cloud_to_scan)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    tf 
    pcl_ros 
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/CloudScan.cfg
)

catkin_package()

include_directories(include ${pcl_ros_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/cloud_to_scan.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

