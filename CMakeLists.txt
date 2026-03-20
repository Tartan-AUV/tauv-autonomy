cmake_minimum_required(VERSION 3.8)
project(tauv_autonomy)

find_package(ament_cmake REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

install(DIRECTORY
  Osprey/urdf
  Osprey/meshes
  Osprey/launch
  DESTINATION share/${PROJECT_NAME}/Osprey
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller.py
  scripts/thruster_allocator.py
  scripts/ned_to_enu_wrench.py
  scripts/force_to_gain.py
  scripts/twist_to_wrench.py
  scripts/play_commands.py
  scripts/waypoint_runner.py
  forceOptimizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
