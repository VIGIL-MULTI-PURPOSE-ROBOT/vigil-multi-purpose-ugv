cmake_minimum_required(VERSION 3.8)
project(vigil_sar)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf meshes config dashboard scripts test DESTINATION share/${PROJECT_NAME})
# ROS 2 nodes (executables)
install(PROGRAMS
  scripts/drive.py scripts/terrain_mapper.py scripts/cliff_navigator.py scripts/ugv_dashboard.py
  scripts/thermal_human_detector.py scripts/human_tracker.py scripts/sar_manager.py scripts/build_sar_world.py
  scripts/find_crash.py scripts/motion_test.py scripts/rover_stability.py scripts/scenario_worlds.py
  scripts/scale_check.py
  DESTINATION lib/${PROJECT_NAME})
# shared Python modules imported by the nodes
install(FILES
  scripts/terrain_core.py scripts/planner_core.py scripts/vision_overlay.py scripts/ros_common.py
  scripts/terrain_window.py scripts/thermal_core.py scripts/sar_core.py scripts/sar_paths.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME sar_offline COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sar_offline.py)
  set_tests_properties(sar_offline PROPERTIES TIMEOUT 900)
endif()
ament_package()
