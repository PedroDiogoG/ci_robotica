#include "pose_control/mission_follower.hpp"

MissionFollowerNode::MissionFollowerNode()
: Node("mission_follower"),
  current_idx_(0),
  state_(State::WAITING_READY),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_theta_(0.0),
  odom_received_(false)
{
    // -------------------------------------------------------------------------
    // Parâmetros (carregados do mission.yaml)
    // -------------------------------------------------------------------------
    this->declare_parameter("waypoints", std::vector<double>{});
    this->declare_parameter("waypoint_wait_time", 1.0);
    this->declare_parameter("pos_tolerance", 0.08);
    this->declare_parameter("yaw_tolerance", 0.10);
    this->declare_parameter("frame_id", std::string("odom"));

    const std::vector<double> flat = this->get_parameter("waypoints").as_double_array();
    wait_time_     = this->get_parameter("waypoint_wait_time").as_double();
    pos_tolerance_ = this->get_parameter("pos_tolerance").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    frame_id_      = this->get_parameter("frame_id").as_string();

    // Formato: [x1, y1, yaw1, x2, y2, yaw2, ...] com no mínimo 3 waypoints
    if (flat.size() % 3 != 0 || flat.size() < 9) {
        RCLCPP_FATAL(this->get_logger(),
            "Parâmetro 'waypoints' inválido: %zu valores. Use [x, y, yaw] por waypoint, "
            "com no mínimo 3 waypoints.", flat.size());
        throw std::runtime_error("waypoints inválidos");
    }
    for (size_t i = 0; i < flat.size(); i += 3) {
        waypoints_.push_back({flat[i], flat[i + 1], flat[i + 2]});
    }

    // -------------------------------------------------------------------------
    // Interfaces ROS 2
    // -------------------------------------------------------------------------
    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MissionFollowerNode::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MissionFollowerNode::mission_loop, this));

    dwell_start_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Missão carregada com %zu waypoints.", waypoints_.size());
}

void MissionFollowerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_       = msg->pose.pose.position.x;
    robot_y_       = msg->pose.pose.position.y;
    robot_theta_   = tf2::getYaw(msg->pose.pose.orientation);
    odom_received_ = true;
}

void MissionFollowerNode::send_current_waypoint()
{
    const Waypoint & wp = waypoints_[current_idx_];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, wp.yaw);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.pose.position.x = wp.x;
    msg.pose.position.y = wp.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    target_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Waypoint %zu/%zu enviado: (%.2f, %.2f, %.2f rad)",
                current_idx_ + 1, waypoints_.size(), wp.x, wp.y, wp.yaw);
}

void MissionFollowerNode::mission_loop()
{
    switch (state_) {
    case State::WAITING_READY:
        // Só inicia quando há odometria e o manager já está inscrito em /target_pose
        if (odom_received_ && target_pub_->get_subscription_count() > 0) {
            send_current_waypoint();
            state_ = State::MOVING;
        }
        break;

    case State::MOVING: {
        const Waypoint & wp = waypoints_[current_idx_];
        const double dist = std::hypot(wp.x - robot_x_, wp.y - robot_y_);
        const double raw_e_yaw = wp.yaw - robot_theta_;
        const double e_yaw = std::atan2(std::sin(raw_e_yaw), std::cos(raw_e_yaw));

        if (dist < pos_tolerance_ && std::fabs(e_yaw) < yaw_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu/%zu alcançado.",
                        current_idx_ + 1, waypoints_.size());
            dwell_start_ = this->now();
            state_ = State::DWELL;
        }
        break;
    }

    case State::DWELL:
        if ((this->now() - dwell_start_).seconds() >= wait_time_) {
            ++current_idx_;
            if (current_idx_ >= waypoints_.size()) {
                RCLCPP_INFO(this->get_logger(), "Missão concluída.");
                state_ = State::DONE;
            } else {
                send_current_waypoint();
                state_ = State::MOVING;
            }
        }
        break;

    case State::DONE:
        timer_->cancel();
        break;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionFollowerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
