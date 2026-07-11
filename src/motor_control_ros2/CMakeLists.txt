cmake_minimum_required(VERSION 3.8)
project(motor_control_ros2)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 消息生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UnitreeGO8010State.msg"
  "msg/UnitreeGO8010Command.msg"
  DEPENDENCIES std_msgs
)

# 包含目录
include_directories(include)

# C++ 库 - 核心逻辑
add_library(motor_control_lib
  src/hardware/serial_interface.cpp
  src/motors/unitree_motor_native.cpp
  src/hardware/config_parser.cpp
)

ament_target_dependencies(motor_control_lib
  rclcpp
  std_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(motor_control_lib
  "${cpp_typesupport_target}"
  yaml-cpp
)

set_target_properties(motor_control_lib PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# C++ 可执行文件 - 电机控制节点（Unitree 串口）
add_executable(motor_control_node
  src/nodes/motor_control_node.cpp
)

target_link_libraries(motor_control_node
  motor_control_lib
)

ament_target_dependencies(motor_control_node
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
)

set_target_properties(motor_control_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# C++ 可执行文件 - 击球控制节点
add_executable(strike_node
  src/nodes/strike_node.cpp
)

target_link_libraries(strike_node
  "${cpp_typesupport_target}"
  yaml-cpp
)

ament_target_dependencies(strike_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

set_target_properties(strike_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# C++ 可执行文件 - 电机监控节点
add_executable(motor_monitor_node
  src/nodes/motor_monitor_node.cpp
)

target_link_libraries(motor_monitor_node
  "${cpp_typesupport_target}"
)

ament_target_dependencies(motor_monitor_node
  rclcpp
  std_msgs
)

set_target_properties(motor_monitor_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)

# 安装目标
install(TARGETS
  motor_control_lib
  motor_control_node
  strike_node
  motor_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
