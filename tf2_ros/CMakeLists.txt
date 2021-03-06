cmake_minimum_required(VERSION 3.5)
project(tf2_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

set(dependencies
  builtin_interfaces
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_action
  tf2
  tf2_msgs
)

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/create_timer_ros.cpp
  src/transform_listener.cpp
  src/buffer_client.cpp
  src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

# buffer_server executable
add_executable(buffer_server src/buffer_server_main.cpp)
target_link_libraries(buffer_server
  ${PROJECT_NAME}
)
ament_target_dependencies(buffer_server
  ${dependencies}
)

add_library(static_transform_broadcaster_node SHARED
  src/static_transform_broadcaster_node.cpp
)
target_compile_definitions(static_transform_broadcaster_node PRIVATE "STATIC_TRANSFORM_BROADCASTER_BUILDING_DLL")
target_include_directories(static_transform_broadcaster_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(static_transform_broadcaster_node ${PROJECT_NAME})
ament_target_dependencies(static_transform_broadcaster_node
  "geometry_msgs"
  "rclcpp"
  "rclcpp_components"
  "tf2_msgs"
)
rclcpp_components_register_nodes(static_transform_broadcaster_node "tf2_ros::StaticTransformBroadcasterNode")

# static_transform_publisher
add_executable(static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
target_link_libraries(static_transform_publisher
  ${PROJECT_NAME}
  static_transform_broadcaster_node
)
ament_target_dependencies(static_transform_publisher
  "geometry_msgs"
  "rclcpp"
  "tf2"
  "tf2_msgs"
)

add_executable(tf2_echo
  src/tf2_echo.cpp
)
target_link_libraries(tf2_echo
  ${PROJECT_NAME}
)
ament_target_dependencies(tf2_echo
  "geometry_msgs"
  "rclcpp"
  "tf2"
)

add_executable(tf2_monitor
  src/tf2_monitor.cpp
)
target_link_libraries(tf2_monitor
  ${PROJECT_NAME}
)
ament_target_dependencies(tf2_monitor
  ${dependencies}
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS static_transform_broadcaster_node
  EXPORT static_transform_broadcaster_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
  buffer_server
  static_transform_publisher
  tf2_echo
  tf2_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(LANGUAGE "c++")

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_buffer test/test_buffer.cpp)
  ament_target_dependencies(test_buffer
    ${dependencies}
  )
  target_link_libraries(test_buffer ${PROJECT_NAME})

  ament_add_gtest(test_buffer_server test/test_buffer_server.cpp)
  ament_target_dependencies(test_buffer_server
    ${dependencies}
  )
  target_link_libraries(test_buffer_server ${PROJECT_NAME})

  ament_add_gtest(test_buffer_client test/test_buffer_client.cpp)
  ament_target_dependencies(test_buffer_client
    ${dependencies}
  )
  target_link_libraries(test_buffer_client ${PROJECT_NAME})

  # Adds a tf2_ros message_filter unittest that uses
  # multiple target frames and a non-zero time tolerance
  ament_add_gtest(${PROJECT_NAME}_test_message_filter test/message_filter_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_message_filter
    ${dependencies}
  )
  target_link_libraries(${PROJECT_NAME}_test_message_filter ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_transform_listener test/test_transform_listener.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_transform_listener
    ${dependencies}
  )
  target_link_libraries(${PROJECT_NAME}_test_transform_listener ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_static_transform_broadcaster test/test_static_transform_broadcaster.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_static_transform_broadcaster
    rclcpp
  )
  target_link_libraries(${PROJECT_NAME}_test_static_transform_broadcaster ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_transform_broadcaster test/test_transform_broadcaster.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_transform_broadcaster
    rclcpp
  )
  target_link_libraries(${PROJECT_NAME}_test_transform_broadcaster ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_time_reset test/time_reset_test.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_time_reset
    ${dependencies}
    rosgraph_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test_time_reset ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_listener test/listener_unittest.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_listener
    builtin_interfaces
    geometry_msgs
    rclcpp
    tf2
  )
  target_link_libraries(${PROJECT_NAME}_test_listener ${PROJECT_NAME})

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} static_transform_broadcaster_node)
ament_export_targets(${PROJECT_NAME} static_transform_broadcaster_node)
ament_export_dependencies(
  ${dependencies}
  rcl_interfaces
  rclcpp_components
)
ament_package()
