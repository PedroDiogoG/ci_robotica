#ifndef POSE_CONTROL_MANAGER_HPP
#define POSE_CONTROL_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "control_toolbox/pid.hpp"

class ManagerNode : public rclcpp::Node
{
public:
    ManagerNode();

private:
    // -------------------------------------------------------------------------
    // Métodos Auxiliares e de Laço
    // -------------------------------------------------------------------------
    void control_loop();
    void publish_velocity(double v, double w);
    void reset_pids();

    // -------------------------------------------------------------------------
    // Callbacks dos Subscribers
    // -------------------------------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    // -------------------------------------------------------------------------
    // Interfaces ROS 2
    // -------------------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // -------------------------------------------------------------------------
    // Controladores PID
    // -------------------------------------------------------------------------
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_theta_;

    // -------------------------------------------------------------------------
    // Variáveis de Estado e Alvo
    // -------------------------------------------------------------------------
    double robot_x_;
    double robot_y_;
    double robot_theta_;

    double target_x_;
    double target_y_;
    double target_theta_;

    // -------------------------------------------------------------------------
    // Parâmetros e Flags Lógicas
    // -------------------------------------------------------------------------
    double max_linear_vel_;
    double max_angular_vel_;
    double pos_tolerance_;
    double yaw_tolerance_;

    bool has_target_;
    rclcpp::Time last_time_;
};

#endif // POSE_CONTROL_MANAGER_HPP
