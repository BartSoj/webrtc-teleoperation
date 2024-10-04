cmake_minimum_required(VERSION 3.16)
project(origin_webrtc_teleop)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../teleop_client_lib
        ${CMAKE_CURRENT_BINARY_DIR}/teleop_client_lib)

add_executable(${PROJECT_NAME}
        src/origin_webrtc_teleop_node_main.cpp
        src/origin_webrtc_teleop_node.cpp
        src/TeleopConfigParser.cpp
)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        nav_msgs
)

target_link_libraries(${PROJECT_NAME}
        teleop_client_lib
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
