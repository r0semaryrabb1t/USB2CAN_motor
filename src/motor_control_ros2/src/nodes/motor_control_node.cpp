#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <atomic>

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/unitree_motor_native.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include "motor_control_ros2/config_parser.hpp"

#include "motor_control_ros2/msg/unitree_go8010_state.hpp"
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <iomanip>
#include <sstream>

namespace motor_control {

class MotorControlNode : public rclcpp::Node {
public:
  MotorControlNode() : Node("motor_control_node") {
    this->declare_parameter("control_frequency", 200.0);
    this->declare_parameter("config_file", "");

    // 加载控制参数
    try {
      std::string control_params_file =
          ament_index_cpp::get_package_share_directory("motor_control_ros2")
          + "/config/control_params.yaml";
      loadControlParams(control_params_file);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "控制参数加载失败: %s，使用默认参数", e.what());
    }

    // 获取配置文件路径
    std::string config_file = this->get_parameter("config_file").as_string();
    if (config_file.empty()) {
      config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2")
                    + "/config/motors.yaml";
    }

    // 初始化串口网络
    serial_network_ = std::make_shared<hardware::SerialNetwork>();

    // 从配置文件初始化电机
    try {
      initializeFromConfig(config_file);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "配置加载失败: %s", e.what());
    }

    // 创建发布者
    unitree_go_state_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010State>(
        "unitree_go8010_states", 10);

    // 创建订阅者
    unitree_go_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010Command>(
        "unitree_go8010_command", 10,
        std::bind(&MotorControlNode::unitreeGOCommandCallback, this, std::placeholders::_1));

    // 启动控制循环
    double control_freq = this->get_parameter("control_frequency").as_double();
    target_control_freq_ = control_freq;
    last_freq_report_time_ = this->now();
    control_timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<int>(1e6 / control_freq)),
        std::bind(&MotorControlNode::controlLoop, this));

    // 启动串口独立通信线程
    startSerialThreads();

    RCLCPP_INFO(this->get_logger(),
        "电机控制节点已启动 - 控制频率: %.1f Hz, 宇树电机: %zu, 串口线程: %zu",
        control_freq, unitree_native_motors_.size(), serial_comm_threads_.size());
  }

  ~MotorControlNode() {
    stopSerialThreads();
    serial_network_->closeAll();
  }

private:
  void initializeFromConfig(const std::string& config_file) {
    SystemConfig config = ConfigParser::loadConfig(config_file);

    // 只初始化串口接口和电机
    for (const auto& serial_config : config.serial_interfaces) {
      std::string interface_name = "serial_" + std::to_string(serial_interfaces_count_++);

      if (!serial_network_->addInterface(interface_name, serial_config.device,
                                         serial_config.baudrate)) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", serial_config.device.c_str());
        continue;
      }

      for (const auto& motor_config : serial_config.motors) {
        addUnitreeNativeMotorFromConfig(motor_config, interface_name, serial_config.device);
      }
    }

    RCLCPP_INFO(this->get_logger(), "配置加载完成 - 宇树电机: %zu",
                unitree_native_motors_.size());
  }

  void addUnitreeNativeMotorFromConfig(const MotorConfig& config,
                                       const std::string& interface_name,
                                       const std::string& device_path) {
    auto motor = std::make_shared<UnitreeMotorNative>(
        config.name, static_cast<uint8_t>(config.id), config.gear_ratio);

    motor->setInterfaceName(interface_name);
    motor->setDevicePath(device_path);

    int direction = (config.direction >= 0) ? 1 : -1;
    unitree_direction_[config.name] = direction;
    unitree_offset_[config.name] = config.offset;

    motors_[config.name] = motor;
    unitree_native_motors_.push_back(motor);

    RCLCPP_INFO(this->get_logger(),
        "添加宇树电机: %s (ID=%d, 方向=%d, offset=%.4f rad, 齿轮比=%.2f) -> %s",
        config.name.c_str(), config.id, direction, config.offset,
        config.gear_ratio, interface_name.c_str());
  }

  // ── 坐标变换 ──────────────────────────────────────────────
  int getUnitreeDirection(const std::string& joint_name) const {
    auto it = unitree_direction_.find(joint_name);
    return (it != unitree_direction_.end()) ? it->second : 1;
  }

  double getUnitreeOffset(const std::string& joint_name) const {
    auto it = unitree_offset_.find(joint_name);
    return (it != unitree_offset_.end()) ? it->second : 0.0;
  }

  double applyUnitreePosition(const std::string& joint_name, double raw_position) const {
    return raw_position * static_cast<double>(getUnitreeDirection(joint_name))
           - getUnitreeOffset(joint_name);
  }

  double applyUnitreeVelocity(const std::string& joint_name, double raw_velocity) const {
    return raw_velocity * static_cast<double>(getUnitreeDirection(joint_name));
  }

  double applyUnitreeTorque(const std::string& joint_name, double raw_torque) const {
    return raw_torque * static_cast<double>(getUnitreeDirection(joint_name));
  }

  double outputToRawUnitreePosition(const std::string& joint_name, double output_position) const {
    int dir = getUnitreeDirection(joint_name);
    double offset = getUnitreeOffset(joint_name);
    return (output_position + offset) / static_cast<double>(dir);
  }

  // ── 主控制循环 ────────────────────────────────────────────
  void controlLoop() {
    control_loop_count_++;
    auto now = this->now();
    double dt = (now - last_freq_report_time_).seconds();
    if (dt >= 1.0) {
      actual_control_freq_ = control_loop_count_ / dt;
      control_loop_count_ = 0;
      last_freq_report_time_ = now;
    }
    publishStates();
  }

  // ── 串口通信线程 ──────────────────────────────────────────
  void serialInterfaceLoop(const std::string& iface_name,
                           std::vector<std::shared_ptr<UnitreeMotorNative>> motors) {
    constexpr size_t FRAME_LEN = 16;
    constexpr size_t BUF_SIZE  = 48;

    RCLCPP_INFO(this->get_logger(),
        "[Serial Thread] 启动: %s, 电机数: %zu", iface_name.c_str(), motors.size());

    std::map<std::string, int> poll_cnt, diag_cnt, fail_cnt;

    while (serial_running_.load(std::memory_order_relaxed)) {
      for (auto& motor : motors) {
        if (!serial_running_.load(std::memory_order_relaxed)) break;

        auto serial = serial_network_->getInterface(iface_name);
        if (!serial || !serial->isOpen()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
              "[Serial Thread] %s: 串口未就绪 (%s)",
              motor->getJointName().c_str(), iface_name.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          continue;
        }

        auto& pcnt = poll_cnt[motor->getJointName()];
        pcnt++;

        uint8_t cmd[17];
        motor->getCommandPacket(cmd);

        uint8_t response[BUF_SIZE];
        ssize_t last_n = 0;

        constexpr int kSerialWaitMs = 2;
        constexpr int kSerialTimeoutMs = 8;
        auto try_recv_parse = [&](uint8_t (&buf)[BUF_SIZE]) -> bool {
          ssize_t n = serial->sendRecvAccumulate(cmd, 17, buf, FRAME_LEN, kSerialWaitMs, kSerialTimeoutMs);
          last_n = n;
          if (n <= 0) return false;
          for (ssize_t off = 0; off + static_cast<ssize_t>(FRAME_LEN) <= n; ++off) {
            if (buf[off] == 0xFD && buf[off + 1] == 0xEE) {
              if (motor->parseFeedback(&buf[off], FRAME_LEN)) return true;
            }
          }
          return false;
        };

        bool ok = try_recv_parse(response);

        auto& dcnt = diag_cnt[motor->getJointName()];
        auto& fails = fail_cnt[motor->getJointName()];
        dcnt++;
        if (!ok) fails++;

        if (!ok) {
          ok = try_recv_parse(response);
          if (!ok) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[Serial Thread] %s: 通信失败 (recv=%zd bytes) iface=%s",
                motor->getJointName().c_str(), last_n, iface_name.c_str());
          }
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "[Serial Thread] 退出: %s", iface_name.c_str());
  }

  void startSerialThreads() {
    if (unitree_native_motors_.empty()) return;

    std::map<std::string, std::vector<std::shared_ptr<UnitreeMotorNative>>> iface_motors;
    for (auto& motor : unitree_native_motors_) {
      iface_motors[motor->getInterfaceName()].push_back(motor);
    }

    serial_running_.store(true, std::memory_order_release);

    for (auto& [name, motors] : iface_motors) {
      serial_comm_threads_.emplace_back(
          &MotorControlNode::serialInterfaceLoop, this, name, motors);
      RCLCPP_INFO(this->get_logger(),
          "[Serial] 为接口 %s 启动独立通信线程（%zu 个电机）", name.c_str(), motors.size());
    }

    RCLCPP_INFO(this->get_logger(),
        "[Serial] 共启动 %zu 个串口并行通信线程", serial_comm_threads_.size());
  }

  void stopSerialThreads() {
    serial_running_.store(false, std::memory_order_release);
    for (auto& t : serial_comm_threads_) {
      if (t.joinable()) t.join();
    }
    serial_comm_threads_.clear();
    RCLCPP_INFO(this->get_logger(), "[Serial] 所有串口通信线程已停止");
  }

  // ── 状态发布 ──────────────────────────────────────────────
  void publishStates() {
    auto now = this->now();
    int64_t current_time_ns = std::chrono::steady_clock::now().time_since_epoch().count();
    double heartbeat_timeout_ms = 500.0;

    for (auto& motor : unitree_native_motors_) {
      motor->checkHeartbeat(heartbeat_timeout_ms, current_time_ns);

      auto msg = motor_control_ros2::msg::UnitreeGO8010State();
      const std::string joint = motor->getJointName();
      msg.header.stamp = now;
      msg.joint_name = joint;
      msg.motor_id = motor->getMotorId();
      msg.online = motor->isOnline();
      msg.position = applyUnitreePosition(joint, motor->getOutputPosition());
      msg.velocity = applyUnitreeVelocity(joint, motor->getOutputVelocity());
      msg.torque = applyUnitreeTorque(joint, motor->getOutputTorque());
      msg.temperature = static_cast<int8_t>(motor->getTemperature());
      msg.error = motor->getErrorCode();
      unitree_go_state_pub_->publish(msg);
    }
  }

  // ── 命令回调 ──────────────────────────────────────────────
  void unitreeGOCommandCallback(const motor_control_ros2::msg::UnitreeGO8010Command::SharedPtr msg) {
    std::vector<std::shared_ptr<UnitreeMotorNative>> matched_motors;
    for (auto& m : unitree_native_motors_) {
      if (m->getMotorId() == msg->id) {
        if (!msg->device.empty() && m->getDevicePath() != msg->device) continue;
        matched_motors.push_back(m);
      }
    }

    if (matched_motors.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "[CMD GO8010] 未找到电机 ID=%d device='%s'", msg->id, msg->device.c_str());
      return;
    }

    switch (msg->mode) {
      case 0:  // MODE_BRAKE
        for (auto& m : matched_motors) m->setBrakeCommand();
        break;

      case 1:  // MODE_FOC
        for (auto& m : matched_motors) {
          const std::string& joint = m->getJointName();
          const int dir = getUnitreeDirection(joint);
          const double raw_pos = outputToRawUnitreePosition(joint, msg->position_target);
          const double raw_vel = msg->velocity_target / static_cast<double>(dir);
          const double raw_tau = msg->torque_ff / static_cast<double>(dir);
          m->setFOCCommand(raw_pos, raw_vel, msg->kp, msg->kd, raw_tau);
        }
        break;

      case 2:  // MODE_CALIBRATE
        for (auto& m : matched_motors) m->setCalibrateCommand();
        break;

      default:
        RCLCPP_WARN(this->get_logger(), "[CMD GO8010] ID=%d 未知模式: %d", msg->id, msg->mode);
        break;
    }
  }

  // ── 控制参数加载 ──────────────────────────────────────────
  void loadControlParams(const std::string& config_file) {
    RCLCPP_INFO(this->get_logger(), "正在加载控制参数: %s", config_file.c_str());
    YAML::Node config = YAML::LoadFile(config_file);

    if (config["motor_control_node"] && config["motor_control_node"]["ros__parameters"]) {
      auto params = config["motor_control_node"]["ros__parameters"];
      if (params["control_frequency"]) {
        double freq = params["control_frequency"].as<double>();
        this->set_parameter(rclcpp::Parameter("control_frequency", freq));
        RCLCPP_INFO(this->get_logger(), "控制频率设置为: %.1f Hz", freq);
      }
    }
  }

  // ── 成员变量 ──────────────────────────────────────────────
  std::shared_ptr<hardware::SerialNetwork> serial_network_;
  std::map<std::string, std::shared_ptr<MotorBase>> motors_;
  std::vector<std::shared_ptr<UnitreeMotorNative>> unitree_native_motors_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr unitree_go_state_pub_;
  rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr unitree_go_cmd_sub_;

  int serial_interfaces_count_ = 0;

  int control_loop_count_ = 0;
  rclcpp::Time last_freq_report_time_;
  double actual_control_freq_ = 0.0;
  double target_control_freq_ = 200.0;

  std::vector<std::thread> serial_comm_threads_;
  std::atomic<bool> serial_running_{false};

  std::unordered_map<std::string, int> unitree_direction_;
  std::unordered_map<std::string, double> unitree_offset_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<motor_control::MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
