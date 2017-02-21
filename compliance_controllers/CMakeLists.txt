cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
project(compliance_controllers)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wl,-z,defs ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp
                                        controller_interface
                                        kdl_parser
                                        compliance_controllers_msgs)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp controller_interface kdl_parser
               INCLUDE_DIRS include 
               LIBRARIES ${PROJECT_NAME}
               DEPENDS orocos_kdl)

include_directories(include 
                    ${catkin_INCLUDE_DIRS} 
                    ${EIGEN_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIRS} 
                    ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

#add_library(reload_contact_sensor_plugin SHARED src/reload_contact_sensor_plugin.cpp)
#target_link_libraries(reload_contact_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_libraries})

add_library(compliance_controllers src/hiqp_time_point.cpp
                                   src/utilities.cpp
                                   src/dummy_position_controller.cpp
                                   src/admittance_controller.cpp
                                   src/impedance_controller.cpp
                                   src/chainjnttojacdotsolver.cpp)
target_link_libraries(compliance_controllers ${catkin_LIBRARIES} 
                                             ${orocos_kdl_LIBRARIES}
                                             ${GAZEBO_libraries})

add_executable(trajectory_publisher src/trajectory_frame_publisher.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})