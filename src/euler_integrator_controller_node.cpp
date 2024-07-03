#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_velocities.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <vector>
#include <stdexcept>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;
    std::vector<int64_t> motor_ids_;

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;
    std::vector<float> desired_velocity_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>::SharedPtr desired_velocity_sub;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    EulerIntegrator()
    : Node("euler_integrator"),
      dt_(this->declare_parameter<double>("dt", 0.1)),
      motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
    {
        if (motor_ids_.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        desired_velocity_.resize(motor_ids_.size(), 0.0);

        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        desired_velocity_sub = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>(
            "/cmd/desired_velocity", 10, std::bind(&EulerIntegrator::desired_velocity_callback, this, std::placeholders::_1));

        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&EulerIntegrator::integrate, this));

        timer_->cancel();
    }

private:
    void integrate()
    {
        for (size_t i = 0; i < motor_positions_.ids.size(); i++)
        {
            motor_positions_.positions[i] += dt_ * desired_velocity_[i]; // Update position based on velocity and time step
        }

        desired_position_pub_->publish(motor_positions_);
    }

    void desired_velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorVelocities::SharedPtr msg)
    {
        for (size_t i = 0; i < motor_ids_.size(); i++)
        {
            auto it = std::find(msg->ids.begin(), msg->ids.end(), motor_ids_[i]);
            if (it != msg->ids.end())
            {
                size_t index = std::distance(msg->ids.begin(), it);
                desired_velocity_[i] = msg->velocities[index];
            }
            else
            {
                RCLCPP_FATAL(this->get_logger(), "ID %ld in the received message is not present in motor_ids_. Exiting...", motor_ids_[i]);
                throw std::runtime_error("ID in the received message is not present in motor_ids_");
            }
        }
    }

    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            if (timer_->is_canceled())
            {
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;
                    for (size_t i = 0; i < motor_ids_.size(); i++)
                    {
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                        }
                        else
                        {
                            RCLCPP_FATAL(this->get_logger(), "Motor ID %ld from parameter 'motor_ids' not found in received motor positions. Exiting...", motor_ids_[i]);
                            throw std::runtime_error("Motor ID from parameter 'motor_ids' not found in received motor positions");
                        }
                    }
                    motor_positions_ = filtered_positions;

                    for (size_t i = 0; i < motor_positions_.positions.size(); i++)
                    {
                        RCLCPP_INFO(this->get_logger(), "ID: %ld, Initial Position: %f", motor_positions_.ids[i], motor_positions_.positions[i]);
                    }

                    timer_->reset();
                    response->success = true;
                    response->message = "Timer started.";
                    RCLCPP_INFO(this->get_logger(), "Timer started.");
                }
                else
                {
                    RCLCPP_FATAL(this->get_logger(), "Failed to receive initial motor positions. Exiting...");
                    throw std::runtime_error("Failed to receive initial motor positions");
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
            if (!timer_->is_canceled())
            {
                timer_->cancel();
                response->success = true;
                response->message = "Timer stopped.";
                RCLCPP_INFO(this->get_logger(), "Timer stopped.");
            }
            else
            {
                response->success = true;
                response->message = "Timer was not running.";
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
