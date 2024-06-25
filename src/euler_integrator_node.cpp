#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    int millisecondsTimer_;
    bool initial_condition_received_;
    bool timer_running_;
    double time_; // Variabile per tracciare il tempo trascorso
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
          timer_running_(false),
          time_(0.0)
    {
        dt_ = this->declare_parameter<double>("dt", 0.1);
        millisecondsTimer_ = this->declare_parameter<int>("millisecondsTimer", 2);

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 1);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(millisecondsTimer_),
            std::bind(&EulerIntegrator::integrate, this));

        timer_->cancel(); // Initially, the timer is stopped
    }

private:
    double sin_fun_(double t)
    {
        double OFF = 2000.0;
        double AMP = 1000.0;
        double F = 1.0;
        return ((AMP * std::sin(2.0 * M_PI * F * t)) + OFF);
    }

    void integrate()
    {
        if (!initial_condition_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial motor positions...");
            return;
        }

        // Increment time
        time_ += dt_;

        // Apply Euler integration at each time step
        for (size_t i = 0; i < positions.size(); i++)
        {
            // Simulate a sinusoidal velocity
            float B = sin_fun_(time_);

            // Update positions (A) using velocities (B)
            positions[i] = positions[i] + dt_ * B;
        }

        // Create a message to publish the new positions
        uclv_seed_robotics_ros_interfaces::msg::MotorPositions desired_positions;
        desired_positions.positions = positions;
        desired_positions.ids = ids;

        // Publish the new positions
        desired_position_pub_->publish(desired_positions);
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (!timer_running_)
            {
                // Wait for the initial condition
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
                    initial_condition_received_ = true;

                    // Set initial positions (A) to motor_positions received
                    positions = motor_positions_.positions;
                    ids = motor_positions_.ids;

                    // Log the initial conditions
                    for (size_t i = 0; i < motor_positions_.positions.size(); i++)
                    {
                        RCLCPP_INFO(this->get_logger(), "ID: %d, Initial Position: %f", motor_positions_.ids[i], motor_positions_.positions[i]);
                    }

                    timer_->reset();
                    timer_running_ = true;
                    response->success = true;
                    response->message = "Timer started.";
                    RCLCPP_INFO(this->get_logger(), "Timer started.");
                }
                else
                {
                    response->success = false;
                    response->message = "Failed to receive initial motor positions.";
                    RCLCPP_WARN(this->get_logger(), "Failed to receive initial motor positions.");
                }
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
