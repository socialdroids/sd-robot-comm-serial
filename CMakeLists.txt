cmake_minimum_required(VERSION 3.5)
project(serial)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

file(GLOB PROTO_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/protofiles/*.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(proto_lib ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(proto_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(proto_lib ${Protobuf_LIBRARIES})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

message("BIN DIR:")
message(${CMAKE_CURRENT_BINARY_DIR})

## Sources
## Add serial library
add_library(${PROJECT_NAME}
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

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
add_executable(serial_example examples/serial_example.cc)
add_dependencies(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example ${PROJECT_NAME} proto_lib ${Protobuf_LIBRARIES})
target_include_directories(serial_example PRIVATE include)

install(TARGETS serial_example proto_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)


## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h ${PROTO_HDRS}
    DESTINATION include/serial
)

# message("Install proto headers")
# install(FILES ${PROTO_HDRS}
#     DESTINATION include/${PROJECT_NAME}
# )
# message("OK")

## Tests
#if(BUILD_TESTING)
#    add_subdirectory(tests)
#endif()

ament_package()
