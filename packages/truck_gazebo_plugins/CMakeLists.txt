cmake_minimum_required(VERSION 3.5)
project(truck_gazebo_plugins)

find_package(ament_cmake REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# odometry

add_library(truck_gazebo_odometry SHARED src/odometry.cpp)

target_include_directories(truck_gazebo_odometry PUBLIC include)
ament_target_dependencies(truck_gazebo_odometry gazebo_dev gazebo_ros nav_msgs rclcpp tf2_msgs)
ament_export_libraries(truck_gazebo_odometry)

# truck_gazebo_occupnacy_grid

add_library(truck_gazebo_occupancy_grid SHARED src/occupancy_grid.cpp)

target_include_directories(truck_gazebo_occupancy_grid PUBLIC include)
ament_target_dependencies(truck_gazebo_occupancy_grid gazebo_dev gazebo_ros  nav_msgs rclcpp)
ament_export_libraries(truck_gazebo_occupancy_grid)


ament_export_include_directories(include)
ament_export_dependencies(gazebo_dev gazebo_ros geometry_msgs nav_msgs rclcpp)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
    truck_gazebo_odometry
    truck_gazebo_occupancy_grid
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_clang_format REQUIRED)
#   find_file(CLANG_FORMAT .clang-format PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../..)
#   ament_clang_format(CONFIG_FILE ${CLANG_FORMAT})
# endif()

ament_package()
