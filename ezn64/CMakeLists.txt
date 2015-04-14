cmake_minimum_required(VERSION 2.8.3)
project(ezn64)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  message_generation
)

include_directories(${catkin_INCLUDE_DIRS}
		    ${PROJECT_SOURCE_DIR}/include/
		    )

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/ezn64_usb_control.cpp
	    ${PROJECT_SOURCE_DIR}/src/ezn64_usb_control_lib.cpp
	    )
	    

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/ezn64_usb_control.h
	    )

add_service_files(FILES reference.srv
                        set_position.srv
                        get_error.srv
                        get_position.srv
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
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES} ${LIBUSB1_LIBRARIES}
)
