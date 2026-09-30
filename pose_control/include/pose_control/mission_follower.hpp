#ifndef POSE_CONTROL_MISSION_FOLLOWER_HPP
#define POSE_CONTROL_MISSION_FOLLOWER_HPP

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Segue uma lista de waypoints sequencialmente: publica um alvo por vez em
// /target_pose (consumido pelo nó "manager") e avança quando a pose em /odom
// entra nas tolerâncias.
class MissionFollowerNode : public rclcpp::Node
{
public:
    MissionFollowerNode();

private:
    struct Waypoint
    {
        double x;
        double y;
        double yaw;
    };

    enum class State
    {
        WAITING_READY,  // espera odometria e o manager estar inscrito
        MOVING,         // waypoint publicado, aguardando chegada
        DWELL,          // chegou, aguardando waypoint_wait_time
        DONE            // missão concluída
    };

    void mission_loop();
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void send_current_waypoint();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<Waypoint> waypoints_;
    size_t current_idx_;
    State state_;

    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool odom_received_;

    double pos_tolerance_;
    double yaw_tolerance_;
    double wait_time_;
    std::string frame_id_;

    rclcpp::Time dwell_start_;
};

#endif // POSE_CONTROL_MISSION_FOLLOWER_HPP
