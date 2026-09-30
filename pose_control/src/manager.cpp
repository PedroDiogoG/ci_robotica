#include "pose_control/manager.hpp"

ManagerNode::ManagerNode() : Node("manager")
{
    // -------------------------------------------------------------------------
    // Parâmetros (carregados do manager.yaml)
    // -------------------------------------------------------------------------
    auto declare_pid = [this](const std::string & name) {
        this->declare_parameter(name + ".p", 0.0);
        this->declare_parameter(name + ".i", 0.0);
        this->declare_parameter(name + ".d", 0.0);
        this->declare_parameter(name + ".i_max", 1.0);
        this->declare_parameter(name + ".i_min", -1.0);
        this->declare_parameter(name + ".antiwindup", true);
    };

    auto init_pid = [this](control_toolbox::Pid & pid, const std::string & name) {
        pid.initPid(
            this->get_parameter(name + ".p").as_double(),
            this->get_parameter(name + ".i").as_double(),
            this->get_parameter(name + ".d").as_double(),
            this->get_parameter(name + ".i_max").as_double(),
            this->get_parameter(name + ".i_min").as_double(),
            this->get_parameter(name + ".antiwindup").as_bool());
    };

    declare_pid("linear");
    declare_pid("lateral");
    declare_pid("angular");

    init_pid(pid_x_, "linear");
    init_pid(pid_y_, "lateral");
    init_pid(pid_theta_, "angular");

    this->declare_parameter("max_linear_vel", 0.5);
    this->declare_parameter("max_angular_vel", 1.5);
    this->declare_parameter("pos_tolerance", 0.05);
    this->declare_parameter("yaw_tolerance", 0.08);

    max_linear_vel_  = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    pos_tolerance_   = this->get_parameter("pos_tolerance").as_double();
    yaw_tolerance_   = this->get_parameter("yaw_tolerance").as_double();

    // -------------------------------------------------------------------------
    // Estado inicial
    // -------------------------------------------------------------------------
    robot_x_ = 0.0;
    robot_y_ = 0.0;
    robot_theta_ = 0.0;
    target_x_ = 0.0;
    target_y_ = 0.0;
    target_theta_ = 0.0;

    has_target_ = false;
    last_time_ = this->now();

    // -------------------------------------------------------------------------
    // Interfaces ROS 2
    // -------------------------------------------------------------------------
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ManagerNode::odom_callback, this, std::placeholders::_1));
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10,
        std::bind(&ManagerNode::target_pose_callback, this, std::placeholders::_1));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ManagerNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Nó Manager iniciado com sucesso.");
}

void ManagerNode::reset_pids()
{
    pid_x_.reset();
    pid_y_.reset();
    pid_theta_.reset();
}

void ManagerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_     = msg->pose.pose.position.x;
    robot_y_     = msg->pose.pose.position.y;
    robot_theta_ = tf2::getYaw(msg->pose.pose.orientation);
}

void ManagerNode::target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    target_x_     = msg->pose.position.x;
    target_y_     = msg->pose.position.y;
    target_theta_ = tf2::getYaw(msg->pose.orientation);

    has_target_ = true;

    reset_pids();
    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Novo alvo recebido: X=%.2f, Y=%.2f, yaw=%.2f",
                target_x_, target_y_, target_theta_);
}

void ManagerNode::control_loop()
{
    // Sem alvo = parado
    if (!has_target_) {
        publish_velocity(0.0, 0.0);
        return;
    }

    // Erro global (do robô até o alvo, em {I})
    const double e_x = target_x_ - robot_x_;
    const double e_y = target_y_ - robot_y_;
    const double raw_e_theta = target_theta_ - robot_theta_;
    const double e_theta = std::atan2(std::sin(raw_e_theta), std::cos(raw_e_theta));

    // Erro rotacionado para o referencial do robô {B}
    const double c = std::cos(robot_theta_);
    const double s = std::sin(robot_theta_);

    const double eb_x     =  c * e_x + s * e_y;
    const double eb_y     = -s * e_x + c * e_y;
    const double eb_theta = e_theta;

    // Critério de chegada: para e espera um novo alvo
    const double dist = std::hypot(e_x, e_y);
    if (dist < pos_tolerance_ && std::fabs(e_theta) < yaw_tolerance_) {
        RCLCPP_INFO(this->get_logger(), "Alvo alcançado (erro: %.3f m, %.3f rad).",
                    dist, e_theta);
        has_target_ = false;
        reset_pids();
        publish_velocity(0.0, 0.0);
        return;
    }

    const rclcpp::Time now = this->now();
    const rclcpp::Duration dt = now - last_time_;
    last_time_ = now;
    if (dt.nanoseconds() <= 0) {
        return;
    }
    const uint64_t dt_ns = static_cast<uint64_t>(dt.nanoseconds());

    // Estratégia 2 (movimento contínuo): eb_x -> v ; eb_y e eb_theta -> w
    /* double v_d = pid_x_.computeCommand(eb_x, dt_ns); */
    /* double w_d = pid_y_.computeCommand(eb_y, dt_ns) */
    /*            + pid_theta_.computeCommand(eb_theta, dt_ns); */


    const double rho = dist;
    double alpha = std::atan2(eb_y, eb_x);
    double dir = 1.0;
    if (std::fabs(alpha) > M_PI_2) {
        dir = -1.0;
        alpha -= std::copysign(M_PI, alpha);
    }

    double v_d = 0.0, w_d = 0.0;
    if (rho >= pos_tolerance_) {
        v_d = dir * pid_x_.computeCommand(rho, dt_ns) * std::cos(alpha);
        w_d = pid_y_.computeCommand(alpha, dt_ns);
    } else {
        w_d = pid_theta_.computeCommand(eb_theta, dt_ns);
    }

    v_d = std::max(-max_linear_vel_, std::min(v_d, max_linear_vel_));
    w_d = std::max(-max_angular_vel_, std::min(w_d, max_angular_vel_));


    publish_velocity(v_d, w_d);
}

void ManagerNode::publish_velocity(double v, double w)
{
    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = v;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = 0.0;

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = w;

    cmd_vel_pub_->publish(twist_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManagerNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
