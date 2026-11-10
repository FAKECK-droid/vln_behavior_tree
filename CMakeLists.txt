cmake_minimum_required(VERSION 3.8)
project(vln_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(vln_agent REQUIRED)
find_package(rgbd_image_queue REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

add_library(vln_submit_event_bt_node SHARED src/event_node_base.cpp)
target_compile_features(vln_submit_event_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_submit_event_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(vln_submit_event_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vln_submit_event_bt_node PUBLIC
  behaviortree_cpp_v3 rclcpp std_msgs tf2_ros)
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

add_library(vln_image_content_utils SHARED src/image_content_utils.cpp)
target_compile_features(vln_image_content_utils PUBLIC cxx_std_17)
target_include_directories(vln_image_content_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vln_image_content_utils PUBLIC behaviortree_cpp_v3)

add_library(vln_get_latest_color_image_bt_node SHARED plugins/tools/get_latest_color_image.cpp)
target_compile_features(vln_get_latest_color_image_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_get_latest_color_image_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(vln_get_latest_color_image_bt_node
  PUBLIC vln_call_tool_bt_node vln_image_content_utils
  PRIVATE vln_agent::vln_agent_utils)

add_library(vln_get_latest_color_image_service_bt_node SHARED plugins/services/get_latest_color_image.cpp)
target_compile_features(vln_get_latest_color_image_service_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_get_latest_color_image_service_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(vln_get_latest_color_image_service_bt_node PUBLIC
  nav2_behavior_tree rclcpp rgbd_image_queue tf2 tf2_ros)
target_link_libraries(vln_get_latest_color_image_service_bt_node
  PUBLIC vln_image_content_utils
  PRIVATE vln_agent::vln_agent_utils)

add_library(vln_submit_surrounding_images_bt_node SHARED plugins/events/submit_surrounding_images.cpp)
target_compile_features(vln_submit_surrounding_images_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_submit_surrounding_images_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(vln_submit_surrounding_images_bt_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vln_submit_surrounding_images_bt_node
  PUBLIC vln_submit_event_bt_node
  PRIVATE vln_agent::vln_agent_utils ${OpenCV_LIBRARIES})

add_library(vln_navigation_event_bt_node SHARED plugins/events/navigation_event.cpp)
target_compile_features(vln_navigation_event_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_navigation_event_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(vln_navigation_event_bt_node PUBLIC geometry_msgs tf2)
target_link_libraries(vln_navigation_event_bt_node
  PUBLIC vln_submit_event_bt_node
  PRIVATE vln_agent::vln_agent_utils)

add_library(vln_goal_expression STATIC third_party/tinyexpr/tinyexpr.c)
set_target_properties(vln_goal_expression PROPERTIES
  POSITION_INDEPENDENT_CODE ON C_STANDARD 99 C_STANDARD_REQUIRED YES C_VISIBILITY_PRESET hidden)
target_compile_definitions(vln_goal_expression PRIVATE TE_POW_FROM_RIGHT)
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  # 保留上游 C 实现的函数指针转换，单独处理第三方源码的 pedantic 提示。
  target_compile_options(vln_goal_expression PRIVATE -Wno-pedantic)
endif()

add_library(vln_update_goal_bt_node SHARED plugins/actions/update_goal.cpp)
target_compile_features(vln_update_goal_bt_node PUBLIC cxx_std_17)
target_compile_definitions(vln_update_goal_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(vln_update_goal_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(vln_update_goal_bt_node SYSTEM PRIVATE third_party/tinyexpr)
ament_target_dependencies(vln_update_goal_bt_node PUBLIC
  behaviortree_cpp_v3 geometry_msgs rclcpp tf2)
target_link_libraries(vln_update_goal_bt_node PRIVATE vln_goal_expression)

install(TARGETS vln_submit_event_bt_node vln_call_tool_bt_node vln_get_latest_color_image_bt_node
  vln_image_content_utils vln_get_latest_color_image_service_bt_node vln_submit_surrounding_images_bt_node
  vln_update_goal_bt_node vln_navigation_event_bt_node
  EXPORT export_vln_event_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY behavior_trees docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES third_party/tinyexpr/LICENSE third_party/tinyexpr/UPSTREAM.md
  DESTINATION share/${PROJECT_NAME}/third_party/tinyexpr)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav_msgs REQUIRED)
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

  ament_add_gtest(test_get_latest_color_image test/test_get_latest_color_image.cpp TIMEOUT 40)
  target_link_libraries(test_get_latest_color_image
    vln_get_latest_color_image_bt_node vln_submit_event_bt_node
    vln_agent::vln_agent_tools vln_agent::vln_agent_utils)
  ament_target_dependencies(test_get_latest_color_image rgbd_image_queue)
  target_compile_definitions(test_get_latest_color_image PRIVATE
    VLN_IMAGE_PLUGIN_PATH="$<TARGET_FILE:vln_get_latest_color_image_bt_node>"
    VLN_IMAGE_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures"
    VLN_IMAGE_AGENT_SHARE="${vln_agent_DIR}/.."
    VLN_EVENT_PLUGIN_PATH="$<TARGET_FILE:vln_submit_event_bt_node>")
  set_tests_properties(test_get_latest_color_image PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=185;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_surrounding_images test/test_surrounding_images.cpp TIMEOUT 30)
  target_include_directories(test_surrounding_images PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_surrounding_images
    vln_submit_surrounding_images_bt_node vln_agent::vln_agent_utils ${OpenCV_LIBRARIES})
  target_compile_definitions(test_surrounding_images PRIVATE
    VLN_MOSAIC_PLUGIN_PATH="$<TARGET_FILE:vln_submit_surrounding_images_bt_node>"
    VLN_MOSAIC_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_surrounding_images PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_get_latest_color_image_service test/test_get_latest_color_image_service.cpp TIMEOUT 40)
  target_include_directories(test_get_latest_color_image_service PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_get_latest_color_image_service
    vln_get_latest_color_image_service_bt_node vln_submit_surrounding_images_bt_node
    vln_agent::vln_agent_utils ${OpenCV_LIBRARIES})
  target_compile_definitions(test_get_latest_color_image_service PRIVATE
    VLN_NATIVE_IMAGE_PLUGIN_PATH="$<TARGET_FILE:vln_get_latest_color_image_service_bt_node>"
    VLN_MOSAIC_PLUGIN_PATH="$<TARGET_FILE:vln_submit_surrounding_images_bt_node>")
  set_tests_properties(test_get_latest_color_image_service PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=189;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_update_goal test/test_update_goal.cpp TIMEOUT 30)
  target_link_libraries(test_update_goal vln_update_goal_bt_node)
  target_compile_definitions(test_update_goal PRIVATE
    VLN_UPDATE_GOAL_PLUGIN_PATH="$<TARGET_FILE:vln_update_goal_bt_node>")
  set_tests_properties(test_update_goal PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=191;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_navigation_event test/test_navigation_event.cpp TIMEOUT 30)
  ament_target_dependencies(test_navigation_event nav_msgs)
  target_link_libraries(test_navigation_event vln_navigation_event_bt_node vln_agent::vln_agent_utils)
  target_compile_definitions(test_navigation_event PRIVATE
    VLN_NAVIGATION_EVENT_PLUGIN_PATH="$<TARGET_FILE:vln_navigation_event_bt_node>")
  set_tests_properties(test_navigation_event PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=197;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  get_filename_component(_nav2_bt_prefix "${nav2_behavior_tree_DIR}/../../.." ABSOLUTE)
  find_library(VLN_TEST_RECOVERY_PLUGIN NAMES nav2_recovery_node_bt_node
    HINTS "${_nav2_bt_prefix}/lib" REQUIRED)
  find_library(VLN_TEST_PIPELINE_PLUGIN NAMES nav2_pipeline_sequence_bt_node
    HINTS "${_nav2_bt_prefix}/lib" REQUIRED)
  find_library(VLN_TEST_RATE_PLUGIN NAMES nav2_rate_controller_bt_node
    HINTS "${_nav2_bt_prefix}/lib" REQUIRED)
  find_library(VLN_TEST_ROUND_ROBIN_PLUGIN NAMES nav2_round_robin_node_bt_node
    HINTS "${_nav2_bt_prefix}/lib" REQUIRED)
  ament_add_gtest(test_lookaround test/test_lookaround.cpp TIMEOUT 40)
  ament_target_dependencies(test_lookaround nav_msgs)
  target_include_directories(test_lookaround PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_lookaround vln_submit_surrounding_images_bt_node
    vln_update_goal_bt_node vln_get_latest_color_image_service_bt_node vln_navigation_event_bt_node
    vln_agent::vln_agent_utils ${OpenCV_LIBRARIES})
  target_compile_definitions(test_lookaround PRIVATE
    VLN_LOOK_NATIVE_PLUGIN="$<TARGET_FILE:vln_get_latest_color_image_service_bt_node>"
    VLN_LOOK_UPDATE_PLUGIN="$<TARGET_FILE:vln_update_goal_bt_node>"
    VLN_LOOK_MOSAIC_PLUGIN="$<TARGET_FILE:vln_submit_surrounding_images_bt_node>"
    VLN_LOOK_NAVIGATION_EVENT_PLUGIN="$<TARGET_FILE:vln_navigation_event_bt_node>"
    VLN_TEST_RECOVERY_PLUGIN="${VLN_TEST_RECOVERY_PLUGIN}"
    VLN_TEST_PIPELINE_PLUGIN="${VLN_TEST_PIPELINE_PLUGIN}"
    VLN_TEST_RATE_PLUGIN="${VLN_TEST_RATE_PLUGIN}"
    VLN_TEST_ROUND_ROBIN_PLUGIN="${VLN_TEST_ROUND_ROBIN_PLUGIN}"
    VLN_LOOKAROUND_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/lookaround.xml"
    VLN_SINGLEGOAL_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/vln_nav2goal.xml"
    VLN_MULTIGOAL_XML="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/vln_navthroughgoals.xml")
  set(_lookaround_test_environment
    "ROS_DOMAIN_ID=195"
    "ROS_LOCALHOST_ONLY=1"
    "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs"
    "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  set_tests_properties(test_lookaround PROPERTIES ENVIRONMENT "${_lookaround_test_environment}")

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_FILE_EXCLUDE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyexpr/tinyexpr.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/tinyexpr/tinyexpr.h")
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_vln_event_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 nav2_behavior_tree nlohmann_json
  rclcpp geometry_msgs std_msgs tf2 tf2_ros vln_agent rgbd_image_queue OpenCV)
ament_package()
