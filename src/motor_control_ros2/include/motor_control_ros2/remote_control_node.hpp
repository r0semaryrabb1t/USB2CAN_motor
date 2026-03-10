#ifndef MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP
#define MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>

namespace motor_control {

/**
 * @brief 遥控器节点
 * 
 * 功能：
 * - 订阅遥控器输入（Joy 消息）
 * - 转换为底盘速度命令（Twist 消息）
 * - 发布到 /cmd_vel 供底盘控制节点使用
 * 
 * 支持的运动：
 * - 前后移动（linear.x）
 * - 左右移动（linear.y）
 * - 旋转（angular.z）
 */
class RemoteControlNode : public rclcpp::Node {
public:
    RemoteControlNode();

private:
    /**
     * @brief 处理遥控器输入
     * 
     * 典型的遥控器映射（以 Xbox 风格手柄为例）：
     * - 左摇杆：前后运动、左右运动
     * - 右摇杆：旋转运动
     * - 按钮：快速/慢速模式切换
     * 
     * @param msg 遥控器输入消息
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * @brief 死区处理
     * 
     * @param value 输入值
     * @param deadzone 死区范围 [0, 1)
     * @return 处理后的值
     */
    double applyDeadzone(double value, double deadzone);

    /**
     * @brief 获取速度缩放因子（根据模式）
     * 
     * @return 缩放因子 [0, 1]
     */
    double getSpeedScaleFactor();

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // 配置参数
    struct Config {
        // 轴映射
        int axis_forward = 1;        // 前后运动轴（通常是左摇杆 Y 轴）
        int axis_strafe = 0;         // 左右运动轴（通常是左摇杆 X 轴）
        int axis_rotate = 3;         // 旋转轴（通常是右摇杆 X 轴）

        // 按钮映射
        int button_speed_up = 5;     // 加速按钮（RB）
        int button_speed_down = 4;   // 减速按钮（LB）
        int button_stop = 6;         // 停止按钮（Back）

        // 速度限制（m/s 和 rad/s）
        double max_linear_velocity = 10.0;    // 最大线速度
        double max_angular_velocity = 3.14;  // 最大角速度

        // 默认速度缩放因子
        double default_speed_scale = 0.5;    // 50% 的最大速度
        double speed_scale_step = 0.5;       // 每次按下/释放快速键调整 50%

        // 死区
        double deadzone = 0.1;               // 摇杆死区范围

        // 发布频率
        double publish_frequency = 50.0;     // Hz
    } config_;

    // 状态
    double current_speed_scale_ = 1.0;  // 当前速度缩放因子 [0, 10.0]，即 0
    geometry_msgs::msg::Twist last_cmd_;  // 上一条命令
    rclcpp::TimerBase::SharedPtr publish_timer_;  // 定时发布定时器
    
    // 按钮状态记录（用于检测按下事件）
    bool last_button_speed_up_ = false;      // 上一帧加速按钮状态
    bool last_button_speed_down_ = false;    // 上一帧减速按钮状态
    bool last_button_stop_ = false;          // 上一帧停止按钮状态

    /**
     * @brief 定时发布最后一条命令
     * 
     * 用于保持持续的速度控制
     */
    void publishTimer();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2_REMOTE_CONTROL_NODE_HPP
