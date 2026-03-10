#pragma once

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include <memory>
#include <cstdint>
#include <cmath>
#include <mutex>

namespace motor_control {

/**
 * @brief CRC-CCITT 查表（与Python实现一致）
 */
extern const uint16_t CRC_TABLE[256];

/**
 * @brief 计算CRC-CCITT (LSB-first)
 */
uint16_t calcCrcCcitt(const uint8_t* data, size_t len);

/**
 * @brief GO-M8010-6 电机命令结构（17字节）
 */
#pragma pack(push, 1)
struct UnitreeNativeCommand {
  uint8_t head[2];      // 0xFE, 0xEE
  uint8_t mode;         // id(4bit) | status(3bit) | none(1bit)
  int16_t tor_des;      // 力矩 q8 format
  int16_t spd_des;      // 速度 q7 format  
  int32_t pos_des;      // 位置 q15 format
  uint16_t k_pos;       // 刚度 q15 format
  uint16_t k_spd;       // 阻尼 q15 format
  uint16_t crc;         // CRC16
};

/**
 * @brief GO-M8010-6 电机反馈结构（16字节）
 */
struct UnitreeNativeFeedback {
  uint8_t head[2];      // 0xFE, 0xEE
  uint8_t mode;         // id(4bit) | status(3bit) | none(1bit)
  int16_t torque;       // 力矩 q8 format
  int16_t speed;        // 速度 q7 format
  int32_t pos;          // 位置 q15 format
  int8_t temp;          // 温度
  uint8_t error_force;  // MError(3bit) | force(12bit) | none(1bit) 的高8位
  uint8_t force_low;    // force 低8位
  uint16_t crc;         // CRC16
};
#pragma pack(pop)

/**
 * @brief 宇树电机原生协议控制类（不依赖SDK）
 * 
 * 支持 GO-M8010-6 电机，使用定点数协议
 * 参考 test_motor_rotate_py.py 实现
 */
class UnitreeMotorNative : public MotorBase {
public:
  // 控制模式
  enum class Mode : uint8_t {
    BRAKE = 0,      // 刹车模式
    FOC = 1,        // FOC闭环控制
    CALIBRATE = 2   // 校准模式
  };
  
  /**
   * @brief 构造函数
   * @param name 电机名称
   * @param motor_id 电机ID (0-15)
   * @param direction 方向 (1 或 -1)
   * @param offset 零点偏移（弧度）
   * @param gear_ratio 齿轮减速比
   * @param k_pos 默认刚度系数
   * @param k_spd 默认阻尼系数
   */
  UnitreeMotorNative(const std::string& name, 
                     uint8_t motor_id,
                     int direction = 1,
                     double offset = 0.0,
                     double gear_ratio = 6.33,
                     double k_pos = 0.2,
                     double k_spd = 0.04);
  
  ~UnitreeMotorNative() override = default;
  
  // ========== MotorBase 纯虚函数实现 ==========
  void updateFeedback(const std::string& interface_name, 
                      uint32_t can_id, 
                      const uint8_t* data, 
                      size_t len) override {
    // 串口电机不使用CAN回调
    (void)interface_name;
    (void)can_id;
    (void)data;
    (void)len;
  }
  
  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override {
    // 串口电机不使用CAN发送
    can_id = 0;
    len = 0;
    (void)data;
  }
  
  void enable() override {
    mode_ = Mode::FOC;
  }
  
  void disable() override {
    mode_ = Mode::BRAKE;
  }
  
  // ========== 状态获取（覆盖基类非虚函数） ==========
  
  /**
   * @brief 获取输出轴位置（弧度）
   */
  double getOutputPosition() const;
  
  /**
   * @brief 获取输出轴速度（弧度/秒）
   */
  double getOutputVelocity() const;
  
  /**
   * @brief 获取输出轴力矩（Nm）
   */
  double getOutputTorque() const;
  
  /**
   * @brief 获取温度
   */
  double getTemperature() const;
  
  /**
   * @brief 获取在线状态
   */
  bool isOnline() const;
  
  /**
   * @brief 检查心跳超时
   */
  void checkHeartbeat(double timeout_ms, int64_t current_time_ns);
  
  // ========== 串口接口 ==========
  
  /**
   * @brief 设置串口接口
   */
  void setSerialInterface(std::shared_ptr<hardware::SerialInterface> serial);
  
  /**
   * @brief 发送命令并接收反馈（同步模式）
   * @return 是否成功收到反馈
   */
  bool sendRecv();
  
  // ========== 控制命令 ==========
  
  /**
   * @brief 设置FOC命令
   * @param pos_des 目标位置（弧度）
   * @param vel_des 目标速度（弧度/秒）
   * @param kp 刚度系数
   * @param kd 阻尼系数
   * @param torque_ff 前馈力矩（Nm）
   */
  void setFOCCommand(double pos_des, double vel_des, 
                     double kp, double kd, double torque_ff);
  
  /**
   * @brief 设置刹车命令
   */
  void setBrakeCommand();
  
  /**
   * @brief 设置校准命令
   */
  void setCalibrateCommand();
  
  /**
   * @brief 设置速度命令（简化接口）
   */
  void setVelocityCommand(double vel_des);
  
  /**
   * @brief 设置位置命令（简化接口）
   */
  void setPositionCommand(double pos_des);
  
  // ========== 获取器 ==========
  uint8_t getMotorId() const { return motor_id_; }
  uint8_t getErrorCode() const { return error_code_; }
  
private:
  /**
   * @brief 构建命令包
   */
  void buildCommandPacket();
  
  /**
   * @brief 解析反馈包
   */
  bool parseFeedbackPacket(const uint8_t* data, size_t len);
  
  // 电机参数
  uint8_t motor_id_;
  int direction_;// 方向 (1 或 -1)
  double offset_;// 零点偏移（弧度）
  double gear_ratio_;
  double default_k_pos_;
  double default_k_spd_;
  
  // 控制命令
  Mode mode_ = Mode::FOC;
  double cmd_pos_des_ = 0.0;    // 目标位置（弧度）
  double cmd_vel_des_ = 0.0;    // 目标速度（弧度/秒）
  double cmd_torque_ff_ = 0.0;  // 前馈力矩（Nm）
  double cmd_kp_ = 0.2;         // 刚度
  double cmd_kd_ = 0.04;        // 阻尼
  
  // 反馈状态
  double fb_position_ = 0.0;    // 当前位置（弧度）
  double fb_velocity_ = 0.0;    // 当前速度（弧度/秒）
  double fb_torque_ = 0.0;      // 当前力矩（Nm）
  int8_t fb_temperature_ = 0;   // 温度
  uint8_t error_code_ = 0;      // 错误码
  
  // 通信
  std::shared_ptr<hardware::SerialInterface> serial_;
  uint8_t tx_buffer_[17];
  uint8_t rx_buffer_[16];
  
  // 状态
  bool online_ = false;
  int64_t last_feedback_time_ns_ = 0;
  mutable std::mutex mutex_;
};

} // namespace motor_control
