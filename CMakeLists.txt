cmake_minimum_required(VERSION 3.8)
project(vln_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vln_agent REQUIRED)

add_library(vln_submit_event_bt_node SHARED src/event_node_base.cpp)
target_compile_features(vln_submit_event_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_submit_event_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(vln_submit_event_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vln_submit_event_bt_node PUBLIC
  behaviortree_cpp_v3 rclcpp std_msgs)
target_link_libraries(vln_submit_event_bt_node
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE vln_agent::vln_agent_utils)

add_library(vln_call_tool_bt_node SHARED src/call_tool_node_base.cpp)
target_compile_features(vln_call_tool_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_call_tool_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(vln_call_tool_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vln_call_tool_bt_node PUBLIC
  behaviortree_cpp_v3 nav2_behavior_tree rclcpp vln_agent)
target_link_libraries(vln_call_tool_bt_node
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE vln_agent::vln_agent_utils)

install(TARGETS vln_submit_event_bt_node vln_call_tool_bt_node
  EXPORT export_vln_event_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_event_node test/test_event_node.cpp TIMEOUT 30)
  target_link_libraries(test_event_node vln_submit_event_bt_node vln_agent::vln_agent_utils)
  target_compile_definitions(test_event_node PRIVATE
    VLN_EVENT_PLUGIN_PATH="$<TARGET_FILE:vln_submit_event_bt_node>")
  set_tests_properties(test_event_node PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=181;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_call_tool_node test/test_call_tool_node.cpp TIMEOUT 40)
  target_link_libraries(test_call_tool_node
    vln_call_tool_bt_node vln_agent::vln_agent_tools vln_agent::vln_agent_utils)
  target_compile_definitions(test_call_tool_node PRIVATE
    VLN_CALL_TOOL_PLUGIN_PATH="$<TARGET_FILE:vln_call_tool_bt_node>"
    VLN_CALL_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_call_tool_node PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=183;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_vln_event_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 nav2_behavior_tree nlohmann_json rclcpp std_msgs vln_agent)
ament_package()
