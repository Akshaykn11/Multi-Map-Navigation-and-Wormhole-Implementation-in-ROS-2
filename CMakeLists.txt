
ament_target_dependencies(multi_map_nav_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  sqlite3
)

# Link the ROS 2 interface typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(multi_map_nav_node ${cpp_typesupport_target})

# Install the executable
install(TARGETS
  multi_map_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs, etc. (if any)
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
