cmake_minimum_required(VERSION 3.5)
project(final_assem2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    config
    launch
    meshes
    urdf
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/balance_controller.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
