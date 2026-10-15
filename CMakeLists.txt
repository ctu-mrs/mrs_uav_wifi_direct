cmake_minimum_required(VERSION 3.16)
project(mrs_uav_wifi_direct)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MSQUIC REQUIRED IMPORTED_TARGET msquic>=2.6.2)

add_executable(mrs_uav_wifi_direct_node src/mrs_uav_wifi_direct.cpp)
target_compile_features(mrs_uav_wifi_direct_node PRIVATE cxx_std_17)
target_compile_options(mrs_uav_wifi_direct_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mrs_uav_wifi_direct_node PkgConfig::MSQUIC yaml-cpp OpenSSL::Crypto Threads::Threads)
ament_target_dependencies(mrs_uav_wifi_direct_node rclcpp)

install(TARGETS mrs_uav_wifi_direct_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
