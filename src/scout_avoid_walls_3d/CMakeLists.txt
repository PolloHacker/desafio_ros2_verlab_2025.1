cmake_minimum_required(VERSION 3.8)
project(scout_avoid_walls_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(avoid_walls src/main.cpp src/wall_avoider.cpp)
ament_target_dependencies(avoid_walls 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  PCL
  pcl_conversions
)
target_link_libraries(avoid_walls ${PCL_LIBRARIES})


install(TARGETS
  avoid_walls
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_wall_avoider_test test/wall_avoider_test.cpp)
  # target_include_directories(${PROJECT_NAME}_wall_avoider_test PUBLIC
  #   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  #   $<INSTALL_INTERFACE:include>
  #   ${PCL_INCLUDE_DIRS}
  # )
  # ament_target_dependencies(${PROJECT_NAME}_wall_avoider_test 
  #   rclcpp 
  #   sensor_msgs 
  #   geometry_msgs 
  #   PCL
  #   pcl_conversions
  # )
  # target_link_libraries(${PROJECT_NAME}_wall_avoider_test ${PCL_LIBRARIES})
endif()

ament_package()
