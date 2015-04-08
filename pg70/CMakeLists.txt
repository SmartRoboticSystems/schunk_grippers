cmake_minimum_required(VERSION 2.8.3)
project(pg70)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation


)

include_directories(${catkin_INCLUDE_DIRS}
		    ${PROJECT_SOURCE_DIR}/include/
		    )

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control.cpp
	    ${PROJECT_SOURCE_DIR}/src/pg70_rs232_control_lib.cpp
	    )
	    

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/pg70_rs232_control.h
	    )

add_service_files(FILES set_position.srv
                        get_state.srv
                        acknowledge_error.srv
                        stop.srv)

generate_messages(DEPENDENCIES)

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  serial

)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
)
