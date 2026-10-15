cmake_minimum_required(VERSION 3.16)
project(mrs_uav_wifi_direct)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NL3 REQUIRED IMPORTED_TARGET libnl-3.0 libnl-genl-3.0)
pkg_check_modules(MSQUIC REQUIRED IMPORTED_TARGET msquic>=2.6.2)

rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/LinkMetrics.msg"
	"msg/TopicStatus.msg"
	"msg/DiscoveredUav.msg"
	"msg/Discovery.msg"
	DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(mrs_uav_wifi_direct_node src/mrs_uav_wifi_direct.cpp)
target_compile_features(mrs_uav_wifi_direct_node PRIVATE cxx_std_17)
target_compile_options(mrs_uav_wifi_direct_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mrs_uav_wifi_direct_node "${cpp_typesupport_target}" PkgConfig::NL3 PkgConfig::MSQUIC yaml-cpp OpenSSL::Crypto Threads::Threads)
ament_target_dependencies(mrs_uav_wifi_direct_node rclcpp)

install(TARGETS mrs_uav_wifi_direct_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
