cmake_minimum_required(VERSION 3.20)

project(amr_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# ============================================================
# Install simulation resources
# ============================================================

install(
  DIRECTORY
    worlds
    models
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# ============================================================
# Package
# ============================================================

ament_package()