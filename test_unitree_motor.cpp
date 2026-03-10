#include "motor_control_ros2/unitree_motor.hpp"
#include <iostream>

int main() {
  // 测试UnitreeMotor构造函数
  motor_control::UnitreeMotor motor("test_motor", 0, 1, 0.0f, 6.33f, 0.20f, 0.04f);
  std::cout << "UnitreeMotor创建成功!" << std::endl;
  std::cout << "电机ID: " << motor.getMotorId() << std::endl;
  return 0;
}