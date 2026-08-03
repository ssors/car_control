cmake_minimum_required(VERSION 3.8)
project(car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
