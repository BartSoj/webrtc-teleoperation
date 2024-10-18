cmake_minimum_required(VERSION 3.7)
project(pc_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../teleop_client_lib
        ${CMAKE_CURRENT_BINARY_DIR}/teleop_client_lib)

add_executable(libdatachannel_app
        main.cpp
        teleop_actions/action.cpp
        teleop_actions/send_counter_action.cpp
        teleop_actions/add_clients_action.cpp
        teleop_actions/stream_video_action.cpp
        teleop_actions/stream_video_cv_action.cpp
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