#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "std_srvs/srv/set_bool.hpp"

// using namespace uclv::dynamixel_utils;

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    int millisecondsTimer_;
    bool initial_condition_received_;
    bool timer_running_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;
    std::vector<float> positions;
    std::vector<uint8_t> ids;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr motor_position_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
        : Node("euler_integrator"),
          initial_condition_received_(false),
          timer_running_(false)
    {
        dt_ = this->declare_parameter<double>("dt", 0.1);
        millisecondsTimer_ = this->declare_parameter<int>("millisecondsTimer", 2);

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

        // Subscription to the motor position topic (for initial condition)
        motor_position_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "motor_state", 1, std::bind(&EulerIntegrator::initialPositionCallback, this, std::placeholders::_1));

        // Publisher for the desired motor position topic
        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 1);

        // Timer for integration
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&EulerIntegrator::integrate, this));

        timer_->cancel(); // Initially, the timer is stopped
    }

private:
    void initialPositionCallback(const uclv_seed_robotics_ros_interfaces::msg::MotorPositions::SharedPtr pos)
    {
        if (!initial_condition_received_)
        {
            positions = pos->positions;
            ids = pos->ids;

            initial_condition_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received initial motor positions.");
        }
    }

    void integrate()
    {
        if (!initial_condition_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial motor positions...");
            return;
        }

        // Euler integration logic
        // For demonstration, the integration logic is omitted
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (!timer_running_)
            {
                timer_->reset();
                timer_running_ = true;
                response->success = true;
                response->message = "Timer started.";
                RCLCPP_INFO(this->get_logger(), "Timer started.");
            }
            else
            {
                response->success = false;
                response->message = "Timer is already running.";
                RCLCPP_WARN(this->get_logger(), "Timer is already running.");
            }
        }
        else
        {
            if (timer_running_)
            {
                timer_->cancel();
                timer_running_ = false;
                response->success = true;
                response->message = "Timer stopped.";
                RCLCPP_INFO(this->get_logger(), "Timer stopped.");
            }
            else
            {
                response->success = false;
                response->message = "Timer is not running.";
                RCLCPP_WARN(this->get_logger(), "Timer is not running.");
            }
        }
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
