cmake_minimum_required(VERSION 3.5)
project(simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf config srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
