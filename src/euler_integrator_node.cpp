#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_positions.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_velocities.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <cmath>
#include <vector>
#include <unordered_map>

/**
 * @brief Class that performs Euler integration on motor positions.
 */
class EulerIntegrator : public rclcpp::Node
{
public:
    double dt_; // Time step for Euler integration
    std::vector<int64_t> motor_ids_; // List of motor IDs to consider
    bool initial_condition_received_; // Flag to check if initial conditions are received
    uclv_seed_robotics_ros_interfaces::msg::MotorPositions motor_positions_; // Current motor positions
    std::vector<float> velocities_; // Current motor velocities
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_; // Service to start/stop integration
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>::SharedPtr velocity_value_sub_; // Subscription to motor velocities
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>::SharedPtr desired_position_pub_; // Publisher for desired motor positions
    rclcpp::TimerBase::SharedPtr timer_; // Timer for periodic integration
    std::unordered_map<int64_t, size_t> motor_id_to_index_; // Map from motor ID to index in motor_positions_

    /**
     * @brief Constructor for the EulerIntegrator class.
     */
    EulerIntegrator()
        : Node("euler_integrator"),
          initial_condition_received_(false)
    {
        // Declare and retrieve parameters
        dt_ = this->declare_parameter<double>("dt", double());
        motor_ids_ = this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>());

        // Create service to start/stop integration
        start_stop_service_ = create_service<std_srvs::srv::SetBool>(
            "startstop", std::bind(&EulerIntegrator::service_callback, this,
                                   std::placeholders::_1, std::placeholders::_2));

        // Subscribe to motor velocities
        velocity_value_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::MotorVelocities>(
            "/cmd/velocity_value", 10,
            std::bind(&EulerIntegrator::velocity_callback, this, std::placeholders::_1));

        // Create publisher for desired motor positions
        desired_position_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
            "desired_position", 10);

        // Create a timer for the integration process
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(dt_),
            std::bind(&EulerIntegrator::integrate, this));

        // Initially, the timer is stopped
        timer_->cancel();
    }

private:
    /**
     * @brief Method to integrate motor positions using Euler integration.
     */
    void integrate()
    {
        // Iterate over the motor IDs specified
        for (const auto& motor_id : motor_ids_)
        {
            // Check if the motor ID is in the map
            auto it = motor_id_to_index_.find(motor_id);
            if (it != motor_id_to_index_.end())
            {
                size_t index = it->second; // Get the index for the motor ID
                // Update the position using Euler integration
                motor_positions_.positions[index] += dt_ * velocities_[index];
            }
        }

        // Publish the updated positions
        desired_position_pub_->publish(motor_positions_);
    }

    /**
     * @brief Callback to handle incoming motor velocities.
     * @param msg Shared pointer to the motor velocities message.
     */
    void velocity_callback(const uclv_seed_robotics_ros_interfaces::msg::MotorVelocities::SharedPtr msg)
    {
        velocities_.clear(); // Clear previous velocities
        // Iterate over the incoming velocities
        for (size_t i = 0; i < msg->ids.size(); i++)
        {
            // Check if the motor ID is one of the specified IDs
            if (motor_id_to_index_.count(msg->ids[i]))
            {
                velocities_.push_back(msg->velocities[i]); // Add the velocity to the list
                std::cout << "id: " << (int)(msg->ids[i]) << " - velocity: " << msg->velocities[i] << "\n";
            }
        }
    }

    /**
     * @brief Callback for the start/stop service.
     * @param request Shared pointer to the service request.
     * @param response Shared pointer to the service response.
     */
    void service_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            // If the timer is stopped, start the integration process
            if (timer_->is_canceled())
            {
                // Wait for the initial conditions to be received
                if (rclcpp::wait_for_message<uclv_seed_robotics_ros_interfaces::msg::MotorPositions>(
                        motor_positions_, this->shared_from_this(), "/motor_state", std::chrono::seconds(1)))
                {
                    initial_condition_received_ = true;
                    velocities_.resize(motor_ids_.size(), 0.0); // Initialize velocities with zeros

                    // Filter motor_positions_ to include only those in motor_ids_
                    uclv_seed_robotics_ros_interfaces::msg::MotorPositions filtered_positions;
                    motor_id_to_index_.clear(); // Clear previous mappings
                    for (size_t i = 0; i < motor_ids_.size(); i++)
                    {
                        auto it = std::find(motor_positions_.ids.begin(), motor_positions_.ids.end(), motor_ids_[i]);
                        if (it != motor_positions_.ids.end())
                        {
                            size_t index = std::distance(motor_positions_.ids.begin(), it);
                            filtered_positions.ids.push_back(motor_positions_.ids[index]);
                            filtered_positions.positions.push_back(motor_positions_.positions[index]);
                            motor_id_to_index_[motor_ids_[i]] = index;
                        }
                    }
                    motor_positions_ = filtered_positions;

                    for (size_t i = 0; i < motor_positions_.positions.size(); i++)
                    {
                        RCLCPP_INFO(this->get_logger(), "ID: %d, Initial Position: %f", motor_positions_.ids[i], motor_positions_.positions[i]);
                    }

                    timer_->reset();
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
