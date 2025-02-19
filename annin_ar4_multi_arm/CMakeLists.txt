cmake_minimum_required(VERSION 3.5)
project(annin_ar4_multi_arm)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
