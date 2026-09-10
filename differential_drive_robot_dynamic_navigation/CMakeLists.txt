cmake_minimum_required(VERSION 3.8)
project(differential_drive_robot_dynamic_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(predicted_obstacle_layer SHARED src/predicted_obstacle_layer.cpp)
target_compile_features(predicted_obstacle_layer PUBLIC cxx_std_17)
ament_target_dependencies(predicted_obstacle_layer
  nav2_costmap_2d pluginlib rclcpp sensor_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d predicted_obstacle_layer.xml)

install(TARGETS predicted_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config maps worlds resource behavior_trees
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  differential_drive_robot_dynamic_navigation/dynamic_obstacle_controller.py
  differential_drive_robot_dynamic_navigation/motion_tracker.py
  differential_drive_robot_dynamic_navigation/mission_runner.py
  differential_drive_robot_dynamic_navigation/check_navigation.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES predicted_obstacle_layer.xml
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md VERIFICATION.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_predicted_obstacle_layer test/test_predicted_obstacle_layer.cpp)
    ament_target_dependencies(test_predicted_obstacle_layer
      nav2_costmap_2d pluginlib rclcpp sensor_msgs)
  endif()
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_dynamic_navigation
      test/test_dynamic_navigation.py)
  endif()
endif()

ament_package()
