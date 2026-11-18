cmake_minimum_required(VERSION 3.8)
project(webots_simulation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY worlds protos resource launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(shutdown test/test_sim_shutdown.py TIMEOUT 60)
  ament_add_pytest_test(diff_drive_commands test/test_diff_drive_commands.py)
  ament_add_pytest_test(diff_drive_odometry test/test_diff_drive_odometry.py)
endif()

install(PROGRAMS kill_sim.sh DESTINATION lib/${PROJECT_NAME})

ament_package()
