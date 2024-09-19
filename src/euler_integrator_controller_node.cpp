#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <vector>
#include <stdexcept>
#include <algorithm>

class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_;  // Time step for integration
    std::vector<int64_t> motor_ids_;  // Motor IDs to be controlled
    std::vector<int64_t> motor_thresholds_;  // Thresholds for motor positions
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr measured_velocity_;  // Latest proportional controller result
    bool measured_velocity_received_;  // Flag to indicate if a proportional result has been received

    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_;  // Current motor positions

    // ROS interfaces
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;  // Service to start/stop the integration
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_sub_;  // Subscription to proportional controller result
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_;  // Publisher for desired motor positions
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic integration updates

    // Topics and services as parameters
    std::string measured_velocity_topic_;
    std::string desired_position_topic_;
    std::string start_stop_service_name_;

    EulerIntegrator()
        : Node("euler_integrator"),
        dt_(this->declare_parameter<double>("dt", 0.001)),  // Get the integration time step (dt) from parameters
        motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),  // Get motor IDs from parameters
        motor_thresholds_(this->declare_parameter<std::vector<int64_t>>("motor_thresholds", {100, 3995})),  // Get motor thresholds from parameters
        measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "measured_velocity")),  // Get proportional result topic from parameters
        desired_position_topic_(this->declare_parameter<std::string>("desired_position_topic", "desired_position")),  // Get desired position topic from parameters
        start_stop_service_name_(this->declare_parameter<std::string>("start_stop_service_name", "startstop")),  // Get start/stop service name from parameters
        measured_velocity_received_(false)  // Initialize flag as false
    {
        // Ensure motor IDs are provided
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        // Ensure motor thresholds are properly set
        if (motor_thresholds_.size() != 2)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_thresholds' must have exactly 2 values. Exiting...");
            throw std::runtime_error("Parameter 'motor_thresholds' must have exactly 2 values.");
        }

        // Create a service to start/stop the integration process
        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            start_stop_service_name_, std::bind(&EulerIntegrator::service_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribe to proportional controller data
        measured_velocity_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 1, std::bind(&EulerIntegrator::measured_velocity_callback, this, std::placeholders::_1));

        // Create a publisher for desired motor positions
        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            desired_position_topic_, 10);

        // Create a timer that calls the integrate method at intervals defined by dt_
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_), std::bind(&EulerIntegrator::integrate, this));

        // Initially, stop the timer to avoid unintended execution
        timer_->cancel();
    }

private:
    // Euler integration function with motor thresholds
    void integrate()
    {
        if (measured_velocity_received_)
        {
            for (size_t i = 0; i < motor_positions_.ids.size(); i++)
            {
                // Find the corresponding motor ID in the proportional result
                auto it = std::find(measured_velocity_->ids.begin(), measured_velocity_->ids.end(), motor_positions_.ids[i]);
                if (it != measured_velocity_->ids.end())
                {
                    size_t index = std::distance(measured_velocity_->ids.begin(), it);
                    // Apply Euler integration based on the error
                    motor_positions_.positions[i] += dt_ * measured_velocity_->data[index];

                    // Anti-WindUp
                    if (motor_positions_.positions[i] > motor_thresholds_[1])
                    {
                        motor_positions_.positions[i] = motor_thresholds_[1];
                    } else if (motor_positions_.positions[i] < motor_thresholds_[0])
                    {
                        motor_positions_.positions[i] = motor_thresholds_[0];
                    }
                }
            }

            // Publish the updated motor positions
            desired_position_pub_->publish(motor_positions_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Proportional result not received yet.");
        }
    }

    // Callback function for receiving proportional controller results
    void measured_velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_velocity_ = msg;  // Store the latest error data
        measured_velocity_received_ = true;  // Set the flag to indicate data has been received
    }

    // Callback function for the start/stop service
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)  // Start the integration process
        {
            if (timer_->is_canceled())  // If the timer is stopped
            {
                // Wait for initial motor positions
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "motor_state", std::chrono::seconds(1)))
                {
                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;

                    // Filter motor positions based on motor_ids_
                    for (int64_t motor_id : motor_ids_)
                    {
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_id);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                        }
                        else
                        {
                            RCLCPP_FATAL(this->get_logger(), "Motor ID %ld from parameter 'motor_ids' not found in received motor positions. Exiting...", motor_id);
                            throw std::runtime_error("Motor ID from parameter 'motor_ids' not found in received motor positions");
                        }
                    }
                    motor_positions_ = filtered_positions;  // Update the current motor positions

                    // Start the timer to begin periodic integration updates
                    timer_->reset();
                    response->success = true;
                    response->message = "Timer started.";
                    RCLCPP_INFO(this->get_logger(), "Timer started.");
                }
                else
                {
                    // If motor positions are not received, log an error and throw an exception
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
        else  // Stop the integration process
        {
            if (!timer_->is_canceled())
            {
                timer_->cancel();  // Stop the timer
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
