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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(std_srvs REQUIRED)



add_executable(talker_2 src/publisher_member_function_2.cpp)
ament_target_dependencies(talker_2 rclcpp std_msgs example_interfaces tf2_ros tf2 geometry_msgs)  # Add this line)

add_executable(listener_2 src/subscriber_member_function_2.cpp)
ament_target_dependencies(listener_2 rclcpp std_msgs)

add_executable(integration_test_node
  src/integration_test_node.cpp
  )
# The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
# to get the default integration test node main function
target_link_libraries(integration_test_node
  catch_ros2::catch_ros2_with_node_main
  )
  
ament_target_dependencies(integration_test_node
  rclcpp std_msgs std_srvs tf2 tf2_ros geometry_msgs
  )
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
  integration_test_node
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
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
endif()

catch_ros2_add_integration_test (ExampleIntegration_TestYAML
  LAUNCH_FILE integration_test.launch.yaml  # example in YAML
  )

ament_package()
