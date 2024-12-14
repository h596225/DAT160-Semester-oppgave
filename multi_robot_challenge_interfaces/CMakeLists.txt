cmake_minimum_required(VERSION 3.5)
project(multi_robot_challenge_interfaces)

# Standarder
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Avhengigheter
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generer interfaces fra .action-filer
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPoint.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# Test-avhengigheter
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
