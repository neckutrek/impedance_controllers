cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(compliance_controllers_testsuite)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wl,-z,defs ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp
                                        controller_interface
                                        kdl_parser
                                        compliance_controllers_msgs
                                        tf)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp controller_interface kdl_parser tf
               INCLUDE_DIRS include 
               LIBRARIES ${PROJECT_NAME}
               DEPENDS orocos_kdl)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(tester src/main.cpp
                      src/finite_state_machine.cpp
                      src/start_pose_generator.cpp
                      src/trajectory_pose_publisher.cpp)
target_link_libraries(tester ${catkin_LIBRARIES})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})