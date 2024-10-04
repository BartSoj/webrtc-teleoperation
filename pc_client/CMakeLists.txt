cmake_minimum_required(VERSION 3.7)
project(pc_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../teleop_client_lib
        ${CMAKE_CURRENT_BINARY_DIR}/teleop_client_lib)

add_executable(libdatachannel_app
        main.cpp
        teleop_actions/Action.cpp
        teleop_actions/SendCounterAction.cpp
        teleop_actions/AddClientsAction.cpp
        teleop_actions/StreamVideoAction.cpp
        teleop_actions/StreamVideoCvAction.cpp
)

target_link_libraries(pc_client
        PRIVATE
        teleop_client_lib
        ${OpenCV_LIBS}
)

target_include_directories(pc_client
        PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)