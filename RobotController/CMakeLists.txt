cmake_minimum_required(VERSION 3.8)
project(FDIA_turtle_enc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lab_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(CONTROL src/controller.cpp
  include/Dyers/he1n.cpp
  include/Dyers/get_prime.cpp
)

# ament_index_register_resource(
#   NAME primes
#   TYPE text
#   FILE include/primes.txt
# )

configure_file(include/Dyers/primes.txt ../../primes.txt)
ament_target_dependencies(CONTROL rclcpp lab_interfaces geometry_msgs nav_msgs tf2 tf2_msgs)


install(TARGETS
  CONTROL
  DESTINATION lib/${PROJECT_NAME})

  # install(TARGETS
  # PLANT
  # DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
