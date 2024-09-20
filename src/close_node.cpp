#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <cmath>
#include <memory>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

class Close : public rclcpp::Node
{
public:
    std::string measured_norm_topic_; // Topic for normalized forces from another node
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_pub_;
    
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Measured normalized forces message
    bool measured_norm_forces_received_ = false; // Flag indicating if data has been received
    
    std::string start_stop_service_name_;
    std::string measured_velocity_topic_;
    std::vector<int64_t> motor_ids_;
    bool service_activated_ = false; // Flag to stop velocity publishing after activating service
    
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr proportional_service_client_; // Client for external service

    // Constructor
    Close()
        : Node("close_node"),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "measured_velocity")),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", {35, 36, 37, 38})),
          start_stop_service_name_(this->declare_parameter<std::string>("start_stop_service_name", "close"))
    {
        // Subscribe to measured normalized forces topic
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&Close::measured_norm_forces_callback, this, std::placeholders::_1));
        
        // Create start/stop service
        start_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
            start_stop_service_name_, std::bind(&Close::service_close_callback, this, _1, std::placeholders::_2));
        
        // Create velocity publisher
        measured_velocity_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 10);
        
        // Client for activating the proportional controller in another node
        proportional_service_client_ = this->create_client<std_srvs::srv::SetBool>("proportional_controller_node/active_proportional");

        // Publish the initial velocity message at startup
        publish_initial_velocity();
    }

private:
    // Function to publish the initial velocity message
    void publish_initial_velocity()
    {
        if (!service_activated_)
        {
            uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped velocity_msg;

            // Assign motor IDs and set initial velocity (e.g., 500)
            for (int64_t motor_id : motor_ids_)
            {
                velocity_msg.ids.push_back(motor_id);
                velocity_msg.data.push_back(500); // Set initial velocity
            }

            RCLCPP_INFO(this->get_logger(), "Publishing initial velocity message");
            measured_velocity_pub_->publish(velocity_msg);
        }
    }

    // Callback for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg; // Store the received message
        measured_norm_forces_received_ = true; // Set the flag indicating data reception
    }

    // Callback for the start/stop service
    void service_close_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        if (request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Service true");

            // Proceed if normalized forces have been received and service has not been activated
            if (measured_norm_forces_received_ && !service_activated_)
            {
                RCLCPP_INFO(this->get_logger(), "Normalized forces received");

                double threshold = 10.0; // Threshold value to compare with forces
                bool all_above_threshold = true;

                // Check if all forces corresponding to motor_ids are above the threshold
                for (size_t i = 0; i < measured_norm_forces_.ids.size(); ++i)
                {
                    int64_t current_id = measured_norm_forces_.ids[i];

                    // If the ID matches one of the motor_ids
                    if (std::find(motor_ids_.begin(), motor_ids_.end(), current_id) != motor_ids_.end())
                    {
                        double data_value = measured_norm_forces_.data[i];
                        if (data_value <= threshold)
                        {
                            all_above_threshold = false;
                            break;
                        }
                    }
                }

                // If all values are above the threshold, activate the proportional controller
                if (all_above_threshold)
                {
                    RCLCPP_INFO(this->get_logger(), "All values above threshold, activating proportional controller");

                    // Create a request to activate the proportional service
                    auto proportional_request = std::make_shared<std_srvs::srv::SetBool::Request>();
                    proportional_request->data = true; // Request to activate

                    // Send the request to the proportional controller service
                    auto result = proportional_service_client_->async_send_request(proportional_request);

                    // Wait for the result and check if the service call was successful
                    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "Proportional controller activated successfully");
                        service_activated_ = true; // Set the flag to prevent further velocity publishing
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to activate proportional controller");
                    }
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service false");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto close_node = std::make_shared<Close>();
        rclcpp::spin(close_node);
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
