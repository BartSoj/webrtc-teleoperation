cmake_minimum_required(VERSION 3.7)
project(libdatachannel_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CLIENT_SOURCES
        robot-client/main.cpp
        robot-client/parse_cl.cpp
        robot-client/parse_cl.h
)
set(GETOPT_SOURCES
        robot-client/getopt.cpp
        robot-client/getopt.h
)

add_subdirectory(deps/json)

find_package(LibDataChannel REQUIRED)
find_package(Threads REQUIRED)

add_executable(libdatachannel_app ${CLIENT_SOURCES})

set_target_properties(libdatachannel_app PROPERTIES
        CXX_STANDARD 17
        OUTPUT_NAME libdatachannel_app)

target_link_libraries(libdatachannel_app LibDataChannel::LibDataChannel Threads::Threads nlohmann_json::nlohmann_json)

if (MSVC)
    add_custom_command(TARGET libdatachannel_app POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE_DIR:datachannel>/datachannel.dll"
            $<TARGET_FILE_DIR:libdatachannel_app>
    )
endif ()