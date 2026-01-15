cmake_minimum_required(VERSION 3.10)
project(rsla_dvl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(PDDecoder
  src/PDDecoder/PDD_Decoder.c
  src/PDDecoder/PDD_Ensemble.c
  src/PDDecoder/PDD_Helper.c)

target_compile_options(PDDecoder PRIVATE
  -Wno-error=incompatible-pointer-types
  -Wno-error=implicit-function-declaration
  -Wno-implicit-function-declaration
  -Wno-builtin-declaration-mismatch
  -Wno-incompatible-pointer-types
  -Wno-unused-parameter
  -Wno-sizeof-pointer-memaccess
  -Wno-sizeof-array-argument
  -Wno-pedantic
  -Wno-extra)

include_directories(
  include
  src/PDDecoder
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rsla_dvl src/main.cpp)
ament_target_dependencies(rsla_dvl std_msgs geometry_msgs rclcpp)

target_link_libraries(rsla_dvl PDDecoder)

install(TARGETS rsla_dvl DESTINATION lib/${PROJECT_NAME})

ament_package()

