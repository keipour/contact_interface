cmake_minimum_required(VERSION 2.8.3)
project(contact_interface)
add_compile_options("-Wno-unused-parameter")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  mav_msgs
  core_px4_interface
  base
)

add_message_files(
  FILES
  ContactCommand.msg
  ContactStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(${PROJECT_NAME}_node src/contact_interface_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install (DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

