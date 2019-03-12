cmake_minimum_required(VERSION 2.8.3)
project(robotiq_common)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

catkin_python_setup()

generate_messages()

catkin_package()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
