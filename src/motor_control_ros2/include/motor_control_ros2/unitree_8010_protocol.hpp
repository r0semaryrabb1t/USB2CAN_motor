#ifndef MOTOR_CONTROL_ROS2__UNITREE_8010_PROTOCOL_HPP_
#define MOTOR_CONTROL_ROS2__UNITREE_8010_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>

namespace motor_control {

/**
 * @brief GO-M8010-6 电机协议定义
 * 
 * 这是SDK版本使用的协议结构，原生协议版本参见 unitree_motor_native.hpp
 */

// 控制模式
enum class GO8010Mode : uint8_t {
  BRAKE = 0,      // 刹车模式
  FOC = 1,        // FOC闭环控制  
  CALIBRATE = 2   // 校准模式
};

/**
 * @brief GO-M8010-6 命令结构
 */
struct GO8010Command {
  uint8_t motor_id = 0;
  GO8010Mode mode = GO8010Mode::FOC;
  float position = 0.0f;      // 目标位置（弧度）
  float velocity = 0.0f;      // 目标速度（弧度/秒）
  float torque = 0.0f;        // 前馈力矩（Nm）
  float kp = 0.2f;            // 刚度
  float kd = 0.04f;           // 阻尼
};

/**
 * @brief GO-M8010-6 反馈结构
 */
struct GO8010Feedback {
  uint8_t motor_id = 0;
  GO8010Mode mode = GO8010Mode::FOC;
  float position = 0.0f;      // 当前位置（弧度）
  float velocity = 0.0f;      // 当前速度（弧度/秒）
  float torque = 0.0f;        // 当前力矩（Nm）
  int8_t temperature = 0;     // 温度
  uint8_t error_code = 0;     // 错误码
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__UNITREE_8010_PROTOCOL_HPP_
