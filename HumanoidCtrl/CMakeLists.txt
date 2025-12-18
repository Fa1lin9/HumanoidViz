cmake_minimum_required(VERSION 3.8)
project(HumanoidCtrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(TARGET_NAME HumanoidCtrl)

# find dependencies
set(PROJECT_DEPENDENCIES
  rclcpp
  ament_cmake
  sensor_msgs
  std_msgs
  humanoid_msgs
  urdf
  # tinyxml2
)

foreach(dep IN LISTS PROJECT_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach(dep IN LISTS PROJECT_DEPENDENCIES)

add_executable(${TARGET_NAME} 
  src/HumanoidCtrl.cpp
  src/main.cpp
)

target_include_directories(${TARGET_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(${TARGET_NAME}
  ${PROJECT_DEPENDENCIES}
)

install(TARGETS ${TARGET_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


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
