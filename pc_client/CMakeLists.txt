cmake_minimum_required(VERSION 3.7)
project(libdatachannel_app)

find_package(LibDataChannel REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
        libavcodec
        libavformat
        libavutil
        libswscale
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(libdatachannel_app
        main.cpp
        ../origin_webrtc_teleop/src/teleop_client/Teleoperation.cpp
        ../origin_webrtc_teleop/src/teleop_client/PeerConnection.cpp
        ../origin_webrtc_teleop/src/teleop_client/VideoEncoder.cpp
        teleop_actions/Action.cpp
        teleop_actions/SendCounterAction.cpp
        teleop_actions/AddClientsAction.cpp
        teleop_actions/StreamVideoAction.cpp
        teleop_actions/StreamVideoCvAction.cpp
)

target_link_libraries(libdatachannel_app
        PRIVATE
        LibDataChannel::LibDataChannel
        Threads::Threads
        nlohmann_json::nlohmann_json
        ${OpenCV_LIBS}
        PkgConfig::FFMPEG
)

target_include_directories(libdatachannel_app
        PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${FFMPEG_INCLUDE_DIRS}
)

target_compile_options(libdatachannel_app PRIVATE ${FFMPEG_CFLAGS})