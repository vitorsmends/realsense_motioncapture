cmake_minimum_required(VERSION 3.8)
project(realsense_motioncapture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(realsense2 REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
)

# Add executables
add_executable(image_filter src/image_filter.cpp)
add_executable(motion_capture src/motion_capture.cpp)
add_executable(depth_image_filter src/depth_image_filter.cpp)

# Link libraries
ament_target_dependencies(image_filter 
  rclcpp 
  sensor_msgs 
  cv_bridge
)

ament_target_dependencies(motion_capture 
  rclcpp
  sensor_msgs 
  cv_bridge 
  geometry_msgs
)

ament_target_dependencies(depth_image_filter 
  rclcpp
  sensor_msgs 
  cv_bridge 
  OpenCV
)

target_link_libraries(image_filter
  ${OpenCV_LIBRARIES}
)

target_link_libraries(motion_capture
  ${OpenCV_LIBRARIES}
  ${realsense2_LIBRARY}
)

target_link_libraries(depth_image_filter
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  image_filter
  motion_capture
  depth_image_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
