cmake_minimum_required(VERSION 3.5)
project(serial)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


## Sources
## Add serial library
add_library(${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/src/serial.cc
    ${CMAKE_SOURCE_DIR}/include/serial/serial.h
    ${CMAKE_SOURCE_DIR}/include/serial/v8stdint.h
)

if(APPLE) # macOS
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/impl/unix.cc
        ${CMAKE_SOURCE_DIR}/src/impl/list_ports/list_ports_osx.cc
    )
     target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX) # .*nix
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/impl/unix.cc
        ${CMAKE_SOURCE_DIR}/src/impl/list_ports/list_ports_linux.cc
     )
         target_link_libraries(${PROJECT_NAME} rt pthread)
elseif(WIN32) # Windows
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/impl/win.cc
        ${CMAKE_SOURCE_DIR}/src/impl/list_ports/list_ports_win.cc
    )
      target_link_libraries(${PROJECT_NAME} setupapi)
endif()


## Include headers
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)


## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

## Install headers
install(FILES ${CMAKE_SOURCE_DIR}/include/serial/serial.h ${CMAKE_SOURCE_DIR}/include/serial/v8stdint.h
    DESTINATION ${CMAKE_SOURCE_DIR}/include/serial
)

## Tests
#if(BUILD_TESTING)
#    add_subdirectory(tests)
#endif()
