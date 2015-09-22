cmake_minimum_required(VERSION 2.8.3)
project(schunk_pg70)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
  tf
  )

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)
                   
SET(SOURCES ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control.cpp
            ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control_lib.cpp)
   
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/pg70_rs232_control.h)

add_service_files(FILES reference.srv
                        set_position.srv
                        get_error.srv
                        get_position.srv
                        acknowledge_error.srv
                        stop.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  DEPENDS roscpp serial tf
  )

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} schunk_pg70_gencpp)
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES})

