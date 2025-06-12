cmake_minimum_required(VERSION 3.5)
project(robot_comm_serial)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  # add_compile_options(-Wall -Wextra -Wpedantic) # Muitos warnings devido ao -Wpedantic
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(include)

# Define a list of dependencies
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  pluginlib
  nav_msgs
  Protobuf
)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

message("Generating protofiles...")
file(GLOB MY_PROTO_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/protofiles/*.proto
)
# protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${MY_PROTO_FILES})
# add_library(proto_lib ${PROTO_SRCS} ${PROTO_HDRS})

foreach(PROTO_FILE ${MY_PROTO_FILES})
    get_filename_component(PROTO_FILENAME ${PROTO_FILE} NAME_WE)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILENAME}.pb.cc
               ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILENAME}.pb.h
        COMMAND ${Protobuf_PROTOC_EXECUTABLE}
        ARGS --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
             --experimental_allow_proto3_optional
             -I ${CMAKE_CURRENT_SOURCE_DIR}/include/protofiles
             ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
    )
    list(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILENAME}.pb.cc)
    list(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/${PROTO_FILENAME}.pb.h)
endforeach()

add_library(proto_lib ${PROTO_SRCS} ${PROTO_HDRS})

target_include_directories(proto_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(proto_lib ${Protobuf_LIBRARIES})

message("Protofiles OK!")

add_subdirectory(cmake/serial)

# add_executable(${PROJECT_NAME} src/robot_comm_serial.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(${PROJECT_NAME} src/robot_comm_serial.cpp src/cobs.c)

target_link_libraries(${PROJECT_NAME} proto_lib ${Protobuf_LIBRARIES} serial)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})


if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_osx.cc
    )
     target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/unix.cc
        src/impl/list_ports/list_ports_linux.cc
     )
         target_link_libraries(${PROJECT_NAME} rt pthread)
elseif(WIN32) # Windows
    target_sources(${PROJECT_NAME} PRIVATE
        src/impl/win.cc
        src/impl/list_ports/list_ports_win.cc
    )
      target_link_libraries(${PROJECT_NAME} setupapi)
    ament_export_libraries(setupapi)
endif()


## Include headers
target_include_directories(${PROJECT_NAME} PRIVATE include)

## Uncomment for example
# add_executable(serial_example examples/serial_example.cc)
# add_dependencies(serial_example ${PROJECT_NAME})
# target_link_libraries(serial_example ${PROJECT_NAME} proto_lib ${Protobuf_LIBRARIES})
# target_include_directories(serial_example PRIVATE include)
#
# install(TARGETS serial_example proto_lib
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
# )


## Install executable
install(TARGETS ${PROJECT_NAME} proto_lib
    DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

# Install YAML Config files
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

## Install headers
install(FILES ${PROTO_HDRS}
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
