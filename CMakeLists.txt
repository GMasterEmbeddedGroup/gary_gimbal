cmake_minimum_required(VERSION 3.5)
project(gary_gimbal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(gimbal_control src/gimbal_control.cpp)

target_include_directories(gimbal_control PRIVATE include)

ament_target_dependencies(
        gimbal_control
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
        control_msgs
        sensor_msgs
)


add_library(gimbal_control_lib SHARED src/gimbal_control.cpp)

target_include_directories(gimbal_control_lib PRIVATE include)

ament_target_dependencies(
        gimbal_control_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
        control_msgs
        sensor_msgs
)

rclcpp_components_register_nodes(gimbal_control_lib "gary_gimbal::GimbalControl")

add_executable(gimbal_autonomous src/gimbal_autonomous.cpp)

target_include_directories(gimbal_autonomous PRIVATE include)

ament_target_dependencies(
        gimbal_autonomous
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
        control_msgs
        sensor_msgs
)


add_library(gimbal_autonomous_lib SHARED src/gimbal_autonomous.cpp)

target_include_directories(gimbal_autonomous_lib PRIVATE include)

ament_target_dependencies(
        gimbal_autonomous_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
        control_msgs
        sensor_msgs
)

rclcpp_components_register_nodes(gimbal_autonomous_lib "gary_gimbal::GimbalAutonomous")


add_executable(gimbal_teleop src/gimbal_teleop.cpp)

target_include_directories(gimbal_teleop PRIVATE include)

ament_target_dependencies(
        gimbal_teleop
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
)


add_library(gimbal_teleop_lib SHARED src/gimbal_teleop.cpp)

target_include_directories(gimbal_teleop_lib PRIVATE include)

ament_target_dependencies(
        gimbal_teleop_lib
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        std_msgs
        gary_msgs
)

rclcpp_components_register_nodes(gimbal_teleop_lib "gary_gimbal::GimbalTeleop")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS gimbal_control gimbal_teleop gimbal_autonomous
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS gimbal_control_lib gimbal_teleop_lib gimbal_autonomous_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        gary_msgs
        std_msgs
        control_msgs
        sensor_msgs
)

ament_package()
