cmake_minimum_required(VERSION 3.7)
project(libdatachannel_app)

add_subdirectory(deps/json)
find_package(LibDataChannel REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(libdatachannel_app robot-client/main.cpp robot-client/Teleoperation.cpp robot-client/PeerConnection.cpp)

target_link_libraries(libdatachannel_app LibDataChannel::LibDataChannel Threads::Threads nlohmann_json::nlohmann_json)