cmake_minimum_required(VERSION 3.7)
project(libdatachannel_app)

find_package(LibDataChannel REQUIRED)
find_package(Threads REQUIRED)
find_package(nlohmann_json REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(libdatachannel_app main.cpp ../origin_webrtc_teleop/src/teleop_client/Teleoperation.cpp ../origin_webrtc_teleop/src/teleop_client/PeerConnection.cpp
        teleop_actions/Action.cpp
        teleop_actions/SendCounterAction.cpp
        teleop_actions/AddClientsAction.cpp
        teleop_actions/StreamVideoAction.cpp)

target_link_libraries(libdatachannel_app LibDataChannel::LibDataChannel Threads::Threads nlohmann_json::nlohmann_json)