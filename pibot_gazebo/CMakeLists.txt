cmake_minimum_required(VERSION 2.8.3)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)
project(pibot_gazebo VERSION 1.0.0)
add_compile_options(-std=c++17)

find_package(robotics_gtest_extension CONFIG REQUIRED)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(/usr/lib/robotics_gtest_extension)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
	std_msgs
  rostest
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(wheel_plugin SHARED wheel_plugin.cpp)
target_link_libraries(wheel_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(ir_plugin SHARED ir_plugin.cpp)
target_link_libraries(ir_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(grabber_plugin SHARED grabber_plugin.cpp)
target_link_libraries(ir_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
    rostest
)

if (CATKIN_ENABLE_TESTING)
  include(CheckIncludeFiles)
  check_include_files(RoboticsGtestExtension.hpp HAVE_ROBOTICS_GTEST_EXTENSION)
  message("HEADER FOUND $HAVE_ROBOTICS_GTEST_EXTENSION")
  # cpp tests
  add_rostest_gtest(cpp_simple
        test/SIM/cpp_simple.test
        test/SIM/cpp_simple.cpp)
  target_link_libraries(cpp_simple ${catkin_LIBRARIES} robotics_gtest_extension)
  # py tests
  #add_rostest(test/SIM/simple.test)
endif()
