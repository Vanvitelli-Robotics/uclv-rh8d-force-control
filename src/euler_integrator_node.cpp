#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"

// using namespace uclv::dynamixel_utils;

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    int millisecondsTimer_;
    bool initial_condition_received_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_position_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          initial_condition_received_(false)
    {
        dt_ = this->declare_parameter<double>("dt", 0.1);
        millisecondsTimer_ = this->declare_parameter<int>("millisecondsTimer", 2);

        // Subscription to the motor position topic (for initial condition)
        motor_position_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "motor_position", 10, std::bind(&EulerIntegrator::motorPositionCallback, this, std::placeholders::_1));

        // Publisher for the desired motor position topic (?)
        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&EulerIntegrator::integrate, this));
    }

private:
    void motorPositionCallback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr msg)
    {
        if (!initial_condition_received_)
        {
            motor_positions_ = *msg;
            initial_condition_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received initial motor positions.");
        }
        else
            return;
    }

    void integrate()
    {
        if (!initial_condition_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial motor positions...");
            return;
        }

        // Euler integration for each motor
        for (size_t i = 0; i < motor_positions_.positions.size(); ++i)
        {
            motor_positions_.positions[i] = motor_positions_.positions[i] + motor_positions_.positions[i] * dt_;
        }

        motor_positions_.header.stamp = this->get_clock()->now();
        desired_position_pub_->publish(motor_positions_);

        RCLCPP_INFO(this->get_logger(), "Published desired motor positions.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto euler_integrator_node = std::make_shared<EulerIntegrator>();
        rclcpp::spin(euler_integrator_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
