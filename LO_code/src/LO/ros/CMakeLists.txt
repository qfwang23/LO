cmake_minimum_required(VERSION 3.16...3.26)
project(LO VERSION 0.3.0 LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/LO/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/LO ${CMAKE_CURRENT_BINARY_DIR}/LO)
endif()

if("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "ROS 1 wrapper will be compiled")

  find_package(
    catkin REQUIRED
    COMPONENTS
      geometry_msgs
      nav_msgs
      sensor_msgs
      roscpp
      rosbag
      std_msgs
      tf2
      tf2_ros
  )

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

catkin_package()


  add_executable(odometry_node OdometryServer.cpp)

  target_compile_features(odometry_node PUBLIC cxx_std_20)
  target_include_directories(odometry_node PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

  target_link_libraries(odometry_node LO::pipeline ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

  install(TARGETS odometry_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

else()
  message(FATAL_ERROR "ROS 1 not found, please ensure ROS 1 is installed and sourced.")
endif()
