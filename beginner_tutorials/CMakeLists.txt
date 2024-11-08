cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(talker_2 src/publisher_member_function_2.cpp)
ament_target_dependencies(talker_2 rclcpp std_msgs example_interfaces)  # Add this line)

add_executable(listener_2 src/subscriber_member_function_2.cpp)
ament_target_dependencies(listener_2 rclcpp std_msgs)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)

install(TARGETS
  talker_2
  listener_2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

target_include_directories(talker_2 PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_include_directories(listener_2 PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)


# Ensure the generated headers are available
install(
  DIRECTORY include/
  DESTINATION include/
)

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



ament_package()
