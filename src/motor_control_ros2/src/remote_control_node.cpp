#include "motor_control_ros2/remote_control_node.hpp"
#include <algorithm>
#include <cmath>

namespace motor_control {

RemoteControlNode::RemoteControlNode() : Node("remote_control_node") {
    RCLCPP_INFO(this->get_logger(), "正在初始化遥控器节点...");
    
    // 声明和读取参数
    this->declare_parameter<int>("axis_forward", config_.axis_forward);
    this->declare_parameter<int>("axis_strafe", config_.axis_strafe);
    this->declare_parameter<int>("axis_rotate", config_.axis_rotate);
    
    this->declare_parameter<int>("button_speed_up", config_.button_speed_up);
    this->declare_parameter<int>("button_speed_down", config_.button_speed_down);
    this->declare_parameter<int>("button_stop", config_.button_stop);
    
    this->declare_parameter<double>("max_linear_velocity", config_.max_linear_velocity);
    this->declare_parameter<double>("max_angular_velocity", config_.max_angular_velocity);
    
    this->declare_parameter<double>("default_speed_scale", config_.default_speed_scale);
    this->declare_parameter<double>("speed_scale_step", config_.speed_scale_step);
    
    this->declare_parameter<double>("deadzone", config_.deadzone);
    this->declare_parameter<double>("publish_frequency", config_.publish_frequency);
    
    // 获取参数值
    config_.axis_forward = this->get_parameter("axis_forward").as_int();
    config_.axis_strafe = this->get_parameter("axis_strafe").as_int();
    config_.axis_rotate = this->get_parameter("axis_rotate").as_int();
    
    config_.button_speed_up = this->get_parameter("button_speed_up").as_int();
    config_.button_speed_down = this->get_parameter("button_speed_down").as_int();
    config_.button_stop = this->get_parameter("button_stop").as_int();
    
    config_.max_linear_velocity = this->get_parameter("max_linear_velocity").as_double();
    config_.max_angular_velocity = this->get_parameter("max_angular_velocity").as_double();
    
    config_.default_speed_scale = this->get_parameter("default_speed_scale").as_double();
    config_.speed_scale_step = this->get_parameter("speed_scale_step").as_double();
    
    config_.deadzone = this->get_parameter("deadzone").as_double();
    config_.publish_frequency = this->get_parameter("publish_frequency").as_double();
    
    current_speed_scale_ = config_.default_speed_scale;
    
    // 创建订阅者
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10,
        std::bind(&RemoteControlNode::joyCallback, this, std::placeholders::_1)
    );
    
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );
    
    // 创建定时发布器
    auto period = std::chrono::duration<double>(1.0 / config_.publish_frequency);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RemoteControlNode::publishTimer, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "遥控器节点启动成功");
    RCLCPP_INFO(this->get_logger(), 
        "轴配置 - 前后: %d, 左右: %d, 旋转: %d",
        config_.axis_forward, config_.axis_strafe, config_.axis_rotate);
    RCLCPP_INFO(this->get_logger(), 
        "按钮配置 - 加速: %d, 减速: %d, 停止: %d",
        config_.button_speed_up, config_.button_speed_down, config_.button_stop);
    RCLCPP_INFO(this->get_logger(), 
        "速度配置 - 线速度上限: %.2f m/s, 角速度上限: %.2f rad/s",
        config_.max_linear_velocity, config_.max_angular_velocity);
    RCLCPP_INFO(this->get_logger(), 
        "默认速度缩放因子: %.1f%%, 死区: %.2f",
        config_.default_speed_scale * 100.0, config_.deadzone);
}

void RemoteControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // 检查按钮索引是否有效
    if (msg->buttons.size() <= static_cast<size_t>(std::max({
        config_.button_speed_up, 
        config_.button_speed_down, 
        config_.button_stop}))) {
        return;
    }
    
    // 检查轴索引是否有效
    if (msg->axes.size() <= static_cast<size_t>(std::max({
        config_.axis_forward, 
        config_.axis_strafe, 
        config_.axis_rotate}))) {
        return;
    }
    
    // 速度缩放调整（检测按钮按下事件，而不是按住）
    // 加速按钮：每按一下增加 10%
    if (msg->buttons[config_.button_speed_up] && !last_button_speed_up_) {
        current_speed_scale_ = std::min(20.0,  // 最大 1000%
            current_speed_scale_ + config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "加速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_up_ = msg->buttons[config_.button_speed_up];
    
    // 减速按钮：每按一下减少 10%
    if (msg->buttons[config_.button_speed_down] && !last_button_speed_down_) {
        current_speed_scale_ = std::max(0.0,  // 最小 0%
            current_speed_scale_ - config_.speed_scale_step);
        RCLCPP_INFO(this->get_logger(), "减速: 速度缩放 = %.0f%%", 
            current_speed_scale_ * 100.0);
    }
    last_button_speed_down_ = msg->buttons[config_.button_speed_down];
    
    // 紧急停止按钮（检测按下事件）
    if (msg->buttons[config_.button_stop] && !last_button_stop_) {
        last_cmd_.linear.x = 0.0;
        last_cmd_.linear.y = 0.0;
        last_cmd_.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "紧急停止");
    }
    last_button_stop_ = msg->buttons[config_.button_stop];
    
    // 读取摇杆输入并应用死区
    double forward = applyDeadzone(msg->axes[config_.axis_forward], config_.deadzone);
    double strafe = applyDeadzone(msg->axes[config_.axis_strafe], config_.deadzone);
    double rotate = applyDeadzone(msg->axes[config_.axis_rotate], config_.deadzone);
    
    // 应用速度缩放和最大速度限制
    double speed_scale = getSpeedScaleFactor();
    
    // 构建速度命令
    last_cmd_.linear.x = forward * config_.max_linear_velocity * speed_scale;
    last_cmd_.linear.y = strafe * config_.max_linear_velocity * speed_scale;
    last_cmd_.angular.z = rotate * config_.max_angular_velocity * speed_scale;
    
    // 立即发布
    cmd_vel_pub_->publish(last_cmd_);
}

double RemoteControlNode::applyDeadzone(double value, double deadzone) {
    // 限制在 [-100, 100] 范围内
    value = std::clamp(value, -100.0, 100.0);
    
    // 应用死区
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    
    // 线性调整：移除死区后进行缩放
    // 如果 value > deadzone，返回 (value - deadzone) / (1 - deadzone)
    // 如果 value < -deadzone，返回 (value + deadzone) / (1 - deadzone)
    if (value > 0) {
        return (value - deadzone) / (1.0 - deadzone);
    } else {
        return (value + deadzone) / (1.0 - deadzone);
    }
}

double RemoteControlNode::getSpeedScaleFactor() {
    return current_speed_scale_;
}

void RemoteControlNode::publishTimer() {
    // 定时发布最后一条命令
    // 这样即使遥控器输入不变，也能持续发送命令信号
    cmd_vel_pub_->publish(last_cmd_);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::RemoteControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
