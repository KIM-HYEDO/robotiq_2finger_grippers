cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f_gripper_rviz)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  pluginlib
  robotiq_2f_gripper_msgs
  roscpp
  rviz
  sensor_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Qt meta-object compiler for the Q_OBJECT panel class
set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib pluginlib robotiq_2f_gripper_msgs roscpp rviz sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/gripper_panel.cpp
  include/${PROJECT_NAME}/gripper_panel.h
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Core Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
