#include "motor_control_ros2/unitree_motor_native.hpp"
#include <cstring>
#include <chrono>
#include <iostream>

namespace motor_control {

// CRC-CCITT 查表（与Python实现完全一致）
const uint16_t CRC_TABLE[256] = {
  0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
  0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
  0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
  0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
  0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
  0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
  0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
  0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
  0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
  0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
  0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
  0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
  0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
  0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
  0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
  0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
  0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
  0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
  0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
  0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
  0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
  0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
  0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
  0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
  0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
  0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
  0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
  0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
  0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
  0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
  0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
  0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

uint16_t calcCrcCcitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ CRC_TABLE[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

UnitreeMotorNative::UnitreeMotorNative(const std::string& name, 
                                       uint8_t motor_id,
                                       int direction,
                                       double offset,
                                       double gear_ratio,
                                       double k_pos,
                                       double k_spd)
  : MotorBase(name, MotorType::UNITREE_GO8010, motor_id, 0)
  , motor_id_(motor_id)
  , direction_(direction)
  , offset_(offset)
  , gear_ratio_(gear_ratio)
  , default_k_pos_(k_pos)
  , default_k_spd_(k_spd)
  , cmd_kp_(k_pos)
  , cmd_kd_(k_spd)
{
  memset(tx_buffer_, 0, sizeof(tx_buffer_));
  memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

double UnitreeMotorNative::getOutputPosition() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fb_position_ * direction_ - offset_;
}

double UnitreeMotorNative::getOutputVelocity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fb_velocity_ * direction_;
}

double UnitreeMotorNative::getOutputTorque() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fb_torque_ * direction_;
}

double UnitreeMotorNative::getTemperature() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<double>(fb_temperature_);
}

bool UnitreeMotorNative::isOnline() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return online_;
}

void UnitreeMotorNative::checkHeartbeat(double timeout_ms, int64_t current_time_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  double dt_ms = (current_time_ns - last_feedback_time_ns_) / 1e6;
  if (dt_ms > timeout_ms) {
    online_ = false;
  }
}

void UnitreeMotorNative::setSerialInterface(std::shared_ptr<hardware::SerialInterface> serial) {
  serial_ = serial;
}

void UnitreeMotorNative::buildCommandPacket() {
  // Header
  tx_buffer_[0] = 0xFE;
  tx_buffer_[1] = 0xEE;
  
  // Mode byte: id(4bit) | status(3bit) | none(1bit)
  // 与Python完全一致: cmd[2] = ((motor_id & 0x0F) | ((mode & 0x07) << 4))
  tx_buffer_[2] = (motor_id_ & 0x0F) | ((static_cast<uint8_t>(mode_) & 0x07) << 4);
  
  // 应用方向和偏移
  double tau_cmd = cmd_torque_ff_ * direction_;
  double spd_cmd = cmd_vel_des_ * direction_;
  double pos_cmd = (cmd_pos_des_ + offset_) * direction_;
  
  // ========== 与Python完全一致的定点数转换 ==========
  // Python: tor_des_q8 = int(max(-32768, min(32767, tau * 256)))
  int16_t tor_des_q8 = static_cast<int16_t>(
    std::max(-32768.0, std::min(32767.0, tau_cmd * 256.0)));
  
  // Python: spd_des_q7 = int(max(-32768, min(32767, spd_des * 256 / 2 / math.pi)))
  // 注意: Python的spd_des输入已经是电机侧弧度/秒，这里需要先转换
  double spd_motor = spd_cmd * gear_ratio_;  // 输出侧 -> 电机侧
  int16_t spd_des_q7 = static_cast<int16_t>(
    std::max(-32768.0, std::min(32767.0, spd_motor * 256.0 / 2.0 / M_PI)));
  
  // Python: pos_des_q15 = int(max(-2147483648, min(2147483647, pos_des * 32768 / 2 / math.pi)))
  // 注意: Python的pos_des输入已经是电机侧弧度，这里需要先转换
  double pos_motor = pos_cmd * gear_ratio_;  // 输出侧 -> 电机侧
  int32_t pos_des_q15 = static_cast<int32_t>(
    std::max(-2147483648.0, std::min(2147483647.0, pos_motor * 32768.0 / 2.0 / M_PI)));
  
  // Python: k_pos_q15 = int(max(0, min(65535, k_pos * 1280))) & 0xFFFF
  uint16_t k_pos_q15 = static_cast<uint16_t>(
    std::max(0.0, std::min(65535.0, cmd_kp_ * 1280.0)));
  
  // Python: k_spd_q15 = int(max(0, min(65535, k_spd * 1280))) & 0xFFFF
  uint16_t k_spd_q15 = static_cast<uint16_t>(
    std::max(0.0, std::min(65535.0, cmd_kd_ * 1280.0)));
  
  // 打包成小端序（与Python struct.pack('<h', ...) 一致） (与 Python struct.pack('<h', ...) 一致)
  memcpy(&tx_buffer_[3], &tor_des_q8, 2);
  memcpy(&tx_buffer_[5], &spd_des_q7, 2);
  memcpy(&tx_buffer_[7], &pos_des_q15, 4);
  memcpy(&tx_buffer_[11], &k_pos_q15, 2);
  memcpy(&tx_buffer_[13], &k_spd_q15, 2);
  
  // CRC (前15字节)
  uint16_t crc = calcCrcCcitt(tx_buffer_, 15);
  tx_buffer_[15] = crc & 0xFF;
  tx_buffer_[16] = (crc >> 8) & 0xFF;
}

bool UnitreeMotorNative::parseFeedbackPacket(const uint8_t* data, size_t len) {
  if (len < 16) {
    return false;
  }
  
  // 检查头部：返回帧头是 0xFD 0xEE（发送帧头是 0xFE 0xEE）
  if (data[0] != 0xFD || data[1] != 0xEE) {
    return false;
  }
  
  // 检查电机ID
  uint8_t fb_id = data[2] & 0x0F;
  if (fb_id != motor_id_) {
    return false;
  }
  
  // 验证CRC
  uint16_t recv_crc = data[14] | (data[15] << 8);
  uint16_t calc_crc = calcCrcCcitt(data, 14);
  if (recv_crc != calc_crc) {
    return false;
  }
  
  // 解析定点数
  int16_t torque_q8;
  int16_t speed_q7;
  int32_t pos_q15;
  int8_t temp;
  
  memcpy(&torque_q8, &data[3], 2);
  memcpy(&speed_q7, &data[5], 2);
  memcpy(&pos_q15, &data[7], 4);
  memcpy(&temp, &data[11], 1);
  
  uint8_t merror = data[12] & 0x07;
  
  // ========== 与 Python 完全一致的转换 ==========
  // Python: torque = torque_q8 / 256.0
  double torque = torque_q8 / 256.0;
  
  // Python: speed = speed_q7 * 2 * math.pi / gear_ratio / 256.0
  double speed = speed_q7 * 2.0 * M_PI / gear_ratio_ / 256.0;
  
  // Python: pos = pos_q15 * 360 / gear_ratio / 32768.0  (输出是度)
  double pos_deg = pos_q15 * 360.0 / gear_ratio_ / 32768.0;
  // 转换为弧度
  double pos_rad = pos_deg * M_PI / 180.0;
  
  // 更新状态
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fb_torque_ = torque;
    fb_velocity_ = speed;
    fb_position_ = pos_rad;
    fb_temperature_ = temp;
    error_code_ = merror;
    online_ = true;
    last_feedback_time_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
  }
  
  return true;
}

bool UnitreeMotorNative::sendRecv() {
  if (!serial_) {
    return false;
  }
  
  // 构建命令包
  buildCommandPacket();
  
  // 调试计数器
  static int send_debug_count = 0;
  send_debug_count++;
  bool debug_print = (send_debug_count % 100 == 1);
  
  if (debug_print) {
    std::cout << "[UnitreeNative TX] ID=" << (int)motor_id_ << " Mode=" << (int)mode_
              << " Data: ";
    for (int i = 0; i < 17; ++i) {
      printf("%02X ", tx_buffer_[i]);
    }
    std::cout << std::endl;
  }
  
  // ========== 与 Python 完全一致的发送接收流程 ==========
  
  // Python: ser.write(cmd)
  ssize_t sent = serial_->send(tx_buffer_, 17);
  if (sent != 17) {
    if (debug_print) {
      std::cerr << "[UnitreeNative] 发送失败: sent=" << sent << std::endl;
    }
    return false;
  }
  
  // Python: time.sleep(0.01) - 等待10ms让电机响应
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // Python: response = b''
  //         while len(response) < 16:
  //             chunk = ser.read(16 - len(response))
  //             if not chunk: break
  //             response += chunk
  uint8_t response[32];
  size_t total_received = 0;
  int timeout_count = 0;
  const int max_timeout = 20;  // 最多等待 20ms
  
  while (total_received < 16 && timeout_count < max_timeout) {
    ssize_t n = serial_->receive(response + total_received, 16 - total_received);
    if (n > 0) {
      total_received += n;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      timeout_count++;
    }
  }
  
  if (debug_print) {
    std::cout << "[UnitreeNative RX] 原始数据 (" << total_received << " 字节): ";
    for (size_t i = 0; i < total_received; ++i) {
      printf("%02X ", response[i]);
    }
    std::cout << std::endl;
  }
  
  // Python: if len(response) >= 16:
  //             feedback = parse_motor_feedback(response)
  if (total_received >= 16) {
    // 直接解析（Python 没有搜索帧头，直接解析前16字节）
    memcpy(rx_buffer_, response, 16);
    
    if (parseFeedbackPacket(rx_buffer_, 16)) {
      if (debug_print) {
        std::cout << "[UnitreeNative] 解析成功" << std::endl;
      }
      return true;
    } else {
      if (debug_print) {
        std::cout << "[UnitreeNative] 解析失败" << std::endl;
      }
    }
  } else {
    if (debug_print) {
      std::cout << "[UnitreeNative] 数据不足: " << total_received << " < 16" << std::endl;
    }
  }
  
  return false;
}

void UnitreeMotorNative::setFOCCommand(double pos_des, double vel_des, 
                                       double kp, double kd, double torque_ff) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::FOC;
  cmd_pos_des_ = pos_des;
  cmd_vel_des_ = vel_des;
  cmd_kp_ = kp;
  cmd_kd_ = kd;
  cmd_torque_ff_ = torque_ff;
}

void UnitreeMotorNative::setBrakeCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::BRAKE;
  cmd_pos_des_ = 0.0;
  cmd_vel_des_ = 0.0;
  cmd_kp_ = 0.0;
  cmd_kd_ = 0.0;
  cmd_torque_ff_ = 0.0;
}

void UnitreeMotorNative::setCalibrateCommand() {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::CALIBRATE;
}

void UnitreeMotorNative::setVelocityCommand(double vel_des) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::FOC;
  cmd_vel_des_ = vel_des;
  cmd_kp_ = 0.0;  // 纯速度控制时刚度为0
  cmd_kd_ = default_k_spd_;
  cmd_torque_ff_ = 0.0;
}

void UnitreeMotorNative::setPositionCommand(double pos_des) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = Mode::FOC;
  cmd_pos_des_ = pos_des;
  cmd_vel_des_ = 0.0;
  cmd_kp_ = default_k_pos_;
  cmd_kd_ = default_k_spd_;
  cmd_torque_ff_ = 0.0;
}

} // namespace motor_control
