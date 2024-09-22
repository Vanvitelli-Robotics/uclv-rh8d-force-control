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
    std::string measured_norm_topic_; // Topic for normalized forces
    std::string start_stop_service_name_; // Name for the start/stop service
    std::string measured_velocity_topic_; // Topic for measured velocity
    std::vector<int64_t> motor_ids_; // Motor IDs for the velocities
    double threshold_; // Threshold for forces
    int64_t initial_velocity_; // Initial velocity value
    
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_velocity_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_stop_service_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr proportional_service_client_;

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; 
    bool measured_norm_forces_received_ = false; 
    bool service_activated_ = false; 

    Close()
        : Node("close_node"),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          measured_velocity_topic_(this->declare_parameter<std::string>("measured_velocity_topic", "measured_velocity")),
                    motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
            motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
          threshold_(this->declare_parameter<double>("threshold", 10.0)),
          initial_velocity_(this->declare_parameter<int64_t>("initial_velocity", 500)),
          start_stop_service_name_(this->declare_parameter<std::string>("start_stop_service_name", "close"))
    {

initialize_motor_to_sensor_map();

        // Subscribe to normalized forces topic
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&Close::measured_norm_forces_callback, this, std::placeholders::_1));

        // Create service for start/stop
        start_stop_service_ = this->create_service<std_srvs::srv::SetBool>(
            start_stop_service_name_, std::bind(&Close::service_close_callback, this, _1, std::placeholders::_2));

        // Create publisher for velocity
        measured_velocity_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_velocity_topic_, 10);

        // Client to activate the proportional controller
        proportional_service_client_ = this->create_client<std_srvs::srv::SetBool>("proportional_controller_node/active_proportional");

        // Publish initial velocity at startup
        publish_initial_velocity();
    }

private:

void initialize_map_from_mappings(
        const std::vector<std::string> &mappings,
        std::unordered_map<int64_t, std::vector<double>> &map,
        const std::string &map_type)
    {
        if (mappings.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is empty or not set. Exiting...", map_type.c_str());
            throw std::runtime_error("Parameter '" + map_type + "' is empty or not set");
        }

        for (const auto &mapping : mappings)
        {
            std::istringstream iss(mapping);
            int64_t key;
            std::vector<double> values;
            char delimiter;

            if (!(iss >> key >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            double value;
            while (iss >> value)
            {
                values.push_back(value);
                iss >> delimiter; // consume the comma if present
            }

            if (values.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No values found for key: %ld", key);
                continue;
            }

            map[key] = values;
            RCLCPP_INFO(this->get_logger(), "Mapped key %ld to values: %s",
                        key, [&values]()
                                                   {
                        std::ostringstream oss;
                        for (size_t i = 0; i < values.size(); ++i)
                        {
                            if (i > 0)
                                oss << ", ";
                            oss << values[i];
                        }
                        return oss.str(); }().c_str());
        }

        if (map.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid mappings were created for '%s'. Exiting...", map_type.c_str());
            throw std::runtime_error("No valid mappings were created for '" + map_type + "'");
        }
    }





void initialize_motor_to_sensor_map()
    {
        initialize_map_from_mappings(motor_sensor_mappings_, motor_to_sensor_map_, "motor_sensor_mappings");
    }

    void publish_initial_velocity()
    {
        if (!service_activated_)
        {
            uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped velocity_msg;

            // Set motor IDs and initial velocity
            for (int64_t motor_id : motor_ids_)
            {
                velocity_msg.ids.push_back(motor_id);
                velocity_msg.data.push_back(initial_velocity_);
            }

            RCLCPP_INFO(this->get_logger(), "Publishing initial velocity: %ld", initial_velocity_);
            measured_velocity_pub_->publish(velocity_msg);
        }
    }

    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;
        measured_norm_forces_received_ = true;
    }

    

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
            bool sensor_3_above_threshold = false;
            bool sensor_4_above_threshold = false;

            // Check if all forces corresponding to motor_ids are above the threshold
            for (size_t i = 0; i < measured_norm_forces_.ids.size(); ++i)
            {
                int64_t current_id = measured_norm_forces_.ids[i];

                // If the ID matches one of the motor_ids
                if (std::find(motor_ids_.begin(), motor_ids_.end(), current_id) != motor_ids_.end())
                {
                    double data_value = measured_norm_forces_.data[i];

                    // Check for sensor 3 and 4 specifically
                    if (current_id == 3)
                    {
                        sensor_3_above_threshold = (data_value > threshold);
                    }
                    else if (current_id == 4)
                    {
                        sensor_4_above_threshold = (data_value > threshold);
                    }
                    else
                    {
                        if (data_value <= threshold)
                        {
                            all_above_threshold = false;
                            break;
                        }
                    }
                }
            }

            // If all values (except for sensor 3 and 4) are above the threshold, 
            // and at least one of sensor 3 or sensor 4 is above the threshold
            if (all_above_threshold && (sensor_3_above_threshold || sensor_4_above_threshold))
            {
                RCLCPP_INFO(this->get_logger(), "All conditions satisfied, activating proportional controller");

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
