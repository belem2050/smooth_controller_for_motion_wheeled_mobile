cmake_minimum_required(VERSION 3.8)
project(diff_wheeled_smooth_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include/diff_wheeled_smooth_ctrl)

add_executable(smooth_ctrl src/smooth_ctrl.cpp src/smooth_ctrl_node.cpp)


target_include_directories(smooth_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(smooth_ctrl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(dependencies
  "rclcpp"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2"
  "std_srvs"
)

ament_target_dependencies(
  smooth_ctrl
  ${dependencies}
)

install(TARGETS smooth_ctrl
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
