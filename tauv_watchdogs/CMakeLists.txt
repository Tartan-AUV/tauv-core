cmake_minimum_required(VERSION 3.8)
project(tauv_watchdogs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tauv_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# watchdog executable
add_executable(watchdog
    src/watchdog.cpp
)
target_include_directories(watchdog PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(watchdog PUBLIC c_std_99 cxx_std_20)

ament_target_dependencies(watchdog
    rclcpp
    tauv_msgs
    std_msgs
    sensor_msgs
    tf2
)

install(TARGETS
    watchdog
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
