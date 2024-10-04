cmake_minimum_required(VERSION 3.7)
project(teleop_client_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(LibDataChannel REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
        libavcodec
        libavformat
        libswscale
        libavutil
)

add_library(teleop_client_lib
        src/PeerConnection.cpp
        src/Teleoperation.cpp
        src/VideoEncoder.cpp
)

target_include_directories(teleop_client_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${FFMPEG_INCLUDE_DIRS}
)

target_link_libraries(teleop_client_lib
        PUBLIC
        LibDataChannel::LibDataChannel
        Threads::Threads
        nlohmann_json::nlohmann_json
        PkgConfig::FFMPEG
)

install(TARGETS teleop_client_lib
        EXPORT teleop_client_lib-targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY include/teleop_client
        DESTINATION include
)