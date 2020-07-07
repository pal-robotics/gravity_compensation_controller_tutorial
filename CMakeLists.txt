cmake_minimum_required(VERSION 2.8.3)
project(gravity_compensation_controller_tutorial)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(CATKIN_COMPONENTS
  controller_interface
  ddynamic_reconfigure
  hardware_interface
  rbdl
  roscpp
  pluginlib
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_COMPONENTS})

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_COMPONENTS}
#  DEPENDS system_lib
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
)


add_library(${PROJECT_NAME} src/gravity_compensation_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES gravity_compensation_controller_tutorial_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

if (CATKIN_ENABLE_TESTING)

    set(TEST_REQUIRED_CATKIN_COMPONENTS
        controller_manager_msgs
        gazebo_msgs
        ${REQUIRED_CATKIN_COMPONENTS})

    find_package(rostest REQUIRED)
    find_package(catkin REQUIRED COMPONENTS ${TEST_REQUIRED_CATKIN_COMPONENTS})

    include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

    add_rostest_gtest(rrbot_gravity_controller_test test/rrbot_gravity_controller_test.test test/rrbot_gravity_controller_test.cpp)
    target_link_libraries(rrbot_gravity_controller_test ${catkin_LIBRARIES} ${PROJECT_NAME})
    add_dependencies(rrbot_gravity_controller_test ${catkin_EXPORTED_TARGETS})

endif(CATKIN_ENABLE_TESTING)

