cmake_minimum_required(VERSION 3.10)
project(scan_deskewer)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_common_interfaces REQUIRED)
find_package(dua_geometry_interfaces REQUIRED)
find_package(dua_node_cpp REQUIRED)
find_package(dua_qos_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(params_manager_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(simple_serviceclient_cpp REQUIRED)
find_package(std_msgs REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/scan_deskewer/params.yaml"
  OUT_FILE "init_parameters.cpp")

# Configure Scan Deskewer node source files
file(GLOB SCAN_DESKEWER_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/deskew/*.cpp"
  "src/scan_deskewer/*.cpp")
list(APPEND SCAN_DESKEWER_SRCS init_parameters.cpp)

# Scan Deskewer
add_library(scan_deskewer SHARED ${SCAN_DESKEWER_SRCS})
target_compile_definitions(scan_deskewer PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(scan_deskewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_deskewer
  Eigen3::Eigen)
target_compile_features(scan_deskewer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(scan_deskewer
  dua_common_interfaces
  dua_geometry_interfaces
  dua_node_cpp
  dua_qos_cpp
  geometry_msgs
  nav_msgs
  params_manager_cpp
  rclcpp
  rclcpp_components
  sensor_msgs
  simple_serviceclient_cpp
  std_msgs)
rclcpp_components_register_nodes(scan_deskewer "scan_deskewer::ScanDeskewer")

# Scan Deskewer standalone application
add_executable(scan_deskewer_app
  src/scan_deskewer_app.cpp)
target_include_directories(scan_deskewer_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(scan_deskewer_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(scan_deskewer_app scan_deskewer)
ament_target_dependencies(scan_deskewer_app
  dua_app_management
  rclcpp)

# Install Scan Deskewer component
install(TARGETS scan_deskewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Scan Deskewer app
install(TARGETS scan_deskewer_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
