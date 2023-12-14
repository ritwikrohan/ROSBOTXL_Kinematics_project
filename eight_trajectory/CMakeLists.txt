cmake_minimum_required(VERSION 3.8)
project(eight_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}  # Add this line to include Eigen
)

add_executable(eight_trajectory src/eight_trajectory.cpp)
target_link_libraries(eight_trajectory ${EIGEN3_LIBRARIES})
ament_target_dependencies(eight_trajectory rclcpp geometry_msgs nav_msgs std_msgs Eigen3 tf2 tf2_geometry_msgs)

install(TARGETS
   eight_trajectory
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
		launch
		DESTINATION share/${PROJECT_NAME})

ament_package()
