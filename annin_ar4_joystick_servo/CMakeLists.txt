cmake_minimum_required(VERSION 3.5)
project(annin_ar4_joystick_servo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/joystick_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
