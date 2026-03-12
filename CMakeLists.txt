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

include(FetchContent)

# --- 
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz)
FetchContent_MakeAvailable(json)

# --- Websocket++
# === boost ===
find_package(Boost REQUIRED COMPONENTS filesystem system thread regex)

FetchContent_Declare(websocketpp
GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
  GIT_TAG 0.8.2)
FetchContent_GetProperties(websocketpp)
if(NOT websocketpp_POPULATED)
  FetchContent_Populate(websocketpp)
  add_subdirectory(${websocketpp_SOURCE_DIR} ${websocketpp_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()
# add interface library with all websocketpp dependencies
add_library(Websockets INTERFACE)
target_include_directories(Websockets INTERFACE ${websocketpp_SOURCE_DIR})
target_link_libraries(Websockets INTERFACE Boost::system Boost::thread Boost::regex)
# ----------------------------------------------------------------------------


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Protobuf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(backward_ros REQUIRED)

find_package(sd_msgs REQUIRED)

find_package(SDL2 REQUIRED)

include_directories(include)
include_directories(${SDL2_INCLUDE_DIRS})

# Define a list of dependencies
set(DEPENDENCIES
  rclcpp
  geometry_msgs
  pluginlib
  nav2_msgs
  nav_msgs
  visualization_msgs 
  sensor_msgs
  std_srvs
  Protobuf
  sd_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
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


# Audio
include(FindPkgConfig)
pkg_check_modules(SDL2_Mixer REQUIRED IMPORTED_TARGET SDL2_mixer)
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})
# Audio

add_subdirectory(cmake/serial)

# add_executable(${PROJECT_NAME} src/robot_comm_serial.cpp ${PROTO_SRCS} ${PROTO_HDRS})
add_executable(${PROJECT_NAME} src/robot_comm_serial.cpp src/websocket_interface.cpp src/cobs.c src/navigation_tester.cpp)

target_link_libraries(${PROJECT_NAME} Websockets proto_lib ${Protobuf_LIBRARIES} serial 
    nlohmann_json::nlohmann_json SDL2::SDL2 PkgConfig::SDL2_Mixer)

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
  DESTINATION include/${PROJECT_NAME}
)

# Install Launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Config files, Web files
install(DIRECTORY
  config web
  DESTINATION share/${PROJECT_NAME}
)

## Install headers
install(FILES ${PROTO_HDRS}
    DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
